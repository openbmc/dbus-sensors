#include "NVMeSubsys.hpp"

#include "NVMePlugin.hpp"
#include "Thresholds.hpp"

#include <filesystem>

void NVMeSubsystem::createStorageAssociation()
{
    std::vector<Association> associations;
    std::filesystem::path p(path);

    associations.emplace_back("chassis", "storage", p.parent_path().string());
    associations.emplace_back("chassis", "drive", p.parent_path().string());
    associations.emplace_back("drive", "storage", path);

    assocIntf = objServer.add_interface(path, association::interface);

    assocIntf->register_property("Associations", associations);

    assocIntf->initialize();
}

// get temporature from a NVMe Basic reading.
static double getTemperatureReading(int8_t reading)
{
    if (reading == static_cast<int8_t>(0x80) ||
        reading == static_cast<int8_t>(0x81))
    {
        // 0x80 = No temperature data or temperature data is more the 5 s
        // old 0x81 = Temperature sensor failure
        return std::numeric_limits<double>::quiet_NaN();
    }

    return reading;
}

NVMeSubsystem::NVMeSubsystem(boost::asio::io_context& io,
                             sdbusplus::asio::object_server& objServer,
                             std::shared_ptr<sdbusplus::asio::connection> conn,
                             const std::string& path, const std::string& name,
                             NVMeIntf intf) :
    io(io),
    objServer(objServer), conn(conn), path(path), name(name), nvmeIntf(intf),
    storage(*dynamic_cast<sdbusplus::bus_t*>(conn.get()), path.c_str()),
    drive(*dynamic_cast<sdbusplus::bus_t*>(conn.get()), path.c_str())
{
    NVMeIntf::Protocol protocol;
    try
    {
        protocol = nvmeIntf.getProtocol();
    }
    catch (const std::runtime_error&)
    {
        throw std::runtime_error("NVMe interface is null");
    }

    // initiate the common interfaces (thermal sensor, Drive and Storage)
    if (protocol != NVMeIntf::Protocol::NVMeBasic &&
        protocol != NVMeIntf::Protocol::NVMeMI)
    {
        throw std::runtime_error("Unsupported NVMe interface");
    }

    /* xyz.openbmc_project.Inventory.Item.Drive */
    drive.protocol(NVMeDrive::DriveProtocol::NVMe);
    drive.type(NVMeDrive::DriveType::SSD);
    // TODO: update capacity

    /* xyz.openbmc_project.Inventory.Item.Storage */
    // make association for Drive/Storage/Chassis
    createStorageAssociation();
}

NVMeSubsystem::~NVMeSubsystem()
{
    objServer.remove_interface(assocIntf);
}

void NVMeSubsystem::start(const SensorData& configData)
{
    // add controllers for the subsystem
    if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeMI)
    {
        auto nvme =
            std::get<std::shared_ptr<NVMeMiIntf>>(nvmeIntf.getInferface());
        nvme->miScanCtrl(
            [self{shared_from_this()}, nvme,
             configData](const std::error_code& ec,
                         const std::vector<nvme_mi_ctrl_t>& ctrlList) mutable {
            if (ec || ctrlList.size() == 0)
            {
                // TODO: mark the subsystem invalid and reschedule refresh
                std::cerr << "fail to scan controllers for the nvme subsystem"
                          << (ec ? ": " + ec.message() : "") << std::endl;
                // self->createStorageAssociation();
                return;
            }

            // TODO: manually open nvme_mi_ctrl_t from cntrl id, instead hacking
            // into structure of nvme_mi_ctrl
            for (auto c : ctrlList)
            {
                /* calucate the cntrl id from nvme_mi_ctrl:
                struct nvme_mi_ctrl
                {
                    struct nvme_mi_ep* ep;
                    __u16 id;
                    struct list_node ep_entry;
                };
                */
                uint16_t* index = reinterpret_cast<uint16_t*>(
                    (reinterpret_cast<uint8_t*>(c) +
                     std::max(sizeof(uint16_t), sizeof(void*))));
                std::filesystem::path path = std::filesystem::path(self->path) /
                                             "controllers" /
                                             std::to_string(*index);
                                             
                try
                {
                    auto nvmeController = std::make_shared<NVMeController>(
                        self->io, self->objServer, self->conn, path.string(),
                        nvme, c);

                    // insert the controllers with empty plugin
                    auto [iter, _] = self->controllers.insert(
                        {*index, {nvmeController, {}}});
                    auto& ctrlPlugin = iter->second.second;

                    // set StorageController Association
                    nvmeController->addSubsystemAssociation(self->path);

                    // creat controller plugin
                    if (self->plugin)
                    {
                        ctrlPlugin = self->plugin->createControllerPlugin(
                            *nvmeController, configData);
                    }
                    nvmeController->start(ctrlPlugin);
                }
                catch (const std::exception& e)
                {
                    std::cerr << "failed to create controller: "
                              << std::to_string(*index)
                              << ", reason: " << e.what() << std::endl;
                }

                index++;
            }
            // self->createStorageAssociation();

            /*
            find primary controller and make association
            The controller is SR-IOV, meaning all controllers (within a
            subsystem) are pointing to a single primary controller. So we
            only need to do identify on an arbatary controller.
            */
            auto ctrl = ctrlList.back();
            nvme->adminIdentify(
                ctrl, nvme_identify_cns::NVME_IDENTIFY_CNS_SECONDARY_CTRL_LIST,
                0, 0,
                [self{self->shared_from_this()}](const std::error_code& ec,
                                                 std::span<uint8_t> data) {
                if (ec || data.size() < sizeof(nvme_secondary_ctrl_list))
                {
                    std::cerr << "fail to identify secondary controller list"
                              << std::endl;
                    return;
                }
                nvme_secondary_ctrl_list& listHdr =
                    *reinterpret_cast<nvme_secondary_ctrl_list*>(data.data());

                // Remove all associations
                for (const auto& [_, pair] : self->controllers)
                {
                    pair.first->setSecAssoc();
                }

                if (listHdr.num == 0)
                {
                    std::cerr << "empty identify secondary controller list"
                              << std::endl;
                    return;
                }

                // all sc_entry pointing to a single pcid, so we only check
                // the first entry.
                auto findPrimary =
                    self->controllers.find(listHdr.sc_entry[0].pcid);
                if (findPrimary == self->controllers.end())
                {
                    std::cerr << "fail to match primary controller from "
                                 "identify sencondary cntrl list"
                              << std::endl;
                    return;
                }
                std::vector<std::shared_ptr<NVMeController>> secCntrls;
                for (int i = 0; i < listHdr.num; i++)
                {
                    auto findSecondary =
                        self->controllers.find(listHdr.sc_entry[i].scid);
                    if (findSecondary == self->controllers.end())
                    {
                        std::cerr << "fail to match secondary controller from "
                                     "identify sencondary cntrl list"
                                  << std::endl;
                        break;
                    }
                    secCntrls.push_back(findSecondary->second.first);
                }
                findPrimary->second.first->setSecAssoc(secCntrls);
                });
        });
    }

    // add thermal sensor for the subsystem
    std::optional<std::string> sensorName = createSensorNameFromPath(path);
    if (!sensorName)
    {
        // fail to parse sensor name from path, using name instead.
        sensorName.emplace(name);
    }

    std::vector<thresholds::Threshold> sensorThresholds;
    if (!parseThresholdsFromConfig(configData, sensorThresholds))
    {
        std::cerr << "error populating thresholds for " << *sensorName << "\n";
        throw std::runtime_error("error populating thresholds for " +
                                 *sensorName);
    }

    ctemp = std::make_shared<NVMeSensor>(objServer, io, conn, *sensorName,
                                         std::move(sensorThresholds), path);
    ctempTimer = std::make_shared<boost::asio::steady_timer>(io);

    // start to poll value for CTEMP sensor.
    if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeBasic)
    {
        auto intf =
            std::get<std::shared_ptr<NVMeBasicIntf>>(nvmeIntf.getInferface());
        ctemp_fetcher_t<NVMeBasicIntf::DriveStatus*> dataFether =
            [intf](std::function<void(const std::error_code&,
                                      NVMeBasicIntf::DriveStatus*)>&& cb) {
            intf->getStatus(std::move(cb));
        };
        ctemp_parser_t<NVMeBasicIntf::DriveStatus*> dataParser =
            [](NVMeBasicIntf::DriveStatus* status) -> std::optional<double> {
            if (status == nullptr)
            {
                return std::nullopt;
            }
            return {getTemperatureReading(status->Temp)};
        };
        pollCtemp(this->ctempTimer, this->ctemp, dataFether, dataParser);
    }
    else if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeMI)
    {
        auto intf =
            std::get<std::shared_ptr<NVMeMiIntf>>(nvmeIntf.getInferface());

        ctemp_fetcher_t<nvme_mi_nvm_ss_health_status*> dataFether =
            [intf](std::function<void(const std::error_code&,
                                      nvme_mi_nvm_ss_health_status*)>&& cb) {
            intf->miSubsystemHealthStatusPoll(std::move(cb));
        };
        ctemp_parser_t<nvme_mi_nvm_ss_health_status*> dataParser =
            [](nvme_mi_nvm_ss_health_status* status) -> std::optional<double> {
            // Drive Functional
            bool df = status->nss & 0x20;
            if (!df)
            {
                return std::nullopt;
            }
            return {getTemperatureReading(status->ctemp)};
        };
        pollCtemp(ctempTimer, ctemp, dataFether, dataParser);
    }

    // TODO: start to poll Drive status.

    if (plugin)
    {
        plugin->start();
    }
}
void NVMeSubsystem::stop()
{
    if (plugin)
    {
        plugin->stop();
    }
    ctempTimer->cancel();
}
