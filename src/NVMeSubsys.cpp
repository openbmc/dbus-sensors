#include "NVMeSubsys.hpp"

#include "NVMePlugin.hpp"
#include "NVMeUtil.hpp"
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
                             const SensorData& configData, NVMeIntf intf) :
    io(io),
    objServer(objServer), conn(conn), path(path), name(name),
    config(configData), nvmeIntf(intf), status(Status::Stop),
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

void NVMeSubsystem::markFunctional(bool toggle)
{
    // disable the subsystem
    if (!toggle)
    {
        if (status == Status::Intiatilzing)
        {
            throw std::runtime_error(
                "cannot stop: the subsystem is intiatilzing");
        }
        status = Status::Stop;
        if (plugin)
        {
            plugin->stop();
        }
        // TODO: the controller should be stopped after controller level polling
        // is enabled

        controllers.clear();
        plugin.reset();
        return;
    }
    if (status == Status::Intiatilzing)
    {
        throw std::runtime_error("cannot start: the subsystem is intiatilzing");
    }
    status = Status::Intiatilzing;

    // add controllers for the subsystem
    if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeMI)
    {
        auto nvme =
            std::get<std::shared_ptr<NVMeMiIntf>>(nvmeIntf.getInferface());
        nvme->miScanCtrl(
            [self{shared_from_this()},
             nvme](const std::error_code& ec,
                   const std::vector<nvme_mi_ctrl_t>& ctrlList) mutable {
            if (ec || ctrlList.size() == 0)
            {
                // TODO: mark the subsystem invalid and reschedule refresh
                std::cerr << "fail to scan controllers for the nvme subsystem"
                          << (ec ? ": " + ec.message() : "") << std::endl;
                self->status = Status::Stop;
                self->markFunctional(false);
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

                    std::shared_ptr<NVMeControllerPlugin> ctrlPlugin;

                    // create controller plugin
                    if (self->plugin)
                    {
                        ctrlPlugin = self->plugin->createControllerPlugin(
                            *nvmeController, self->config);
                    }

                    // insert the controllers and controller plugin
                    self->controllers.insert(
                        {*index, {nvmeController, ctrlPlugin}});

                    // set StorageController Association
                    nvmeController->addSubsystemAssociation(self->path);
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
                nvme_secondary_ctrl_list* listHdr = nullptr;
                int num_of_identifiers = 0;
                if (ec || data.size() < sizeof(nvme_secondary_ctrl_list))
                {
                    std::cerr << "failed to identify secondary controller list"
                              << std::endl;
                }
                else
                {
                    listHdr = reinterpret_cast<nvme_secondary_ctrl_list*>(data.data());
                    num_of_identifiers = listHdr->num;
                }

                // Remove all associations
                for (const auto& [_, pair] : self->controllers)
                {
                    pair.first->setSecAssoc();
                }

                auto findPrimary = self->controllers.begin();
                if (num_of_identifiers != 0)
                {
                    // all sc_entry pointing to a single pcid, so we only check
                    // the first entry.
                    findPrimary = self->controllers.find(listHdr->sc_entry[0].pcid);
                    if (findPrimary == self->controllers.end())
                    {
                        std::cerr << "failed to match primary controller from "
                                     "identify sencondary cntrl list"
                                  << std::endl;
                        self->status = Status::Stop;
                        self->markFunctional(false);
                        return;
                    }
                }

                // Enable primary controller since they are required to work
                auto& primaryController = findPrimary->second.first;
                primaryController = NVMeControllerEnabled::create(
                    std::move(*primaryController.get()));

                std::vector<std::shared_ptr<NVMeController>> secCntrls;
                for (int i = 0; i < num_of_identifiers; i++)
                {
                    auto findSecondary =
                        self->controllers.find(listHdr->sc_entry[i].scid);
                    if (findSecondary == self->controllers.end())
                    {
                        std::cerr << "failed to match secondary controller from "
                                     "identify sencondary cntrl list"
                                  << std::endl;
                        break;
                    }

                    auto& secondaryController = findSecondary->second.first;

                    // Check Secondary Controller State
                    if (listHdr->sc_entry[i].scs != 0)
                    {
                        secondaryController = NVMeControllerEnabled::create(
                            std::move(*secondaryController.get()));
                    }
                    secCntrls.push_back(secondaryController);
                }
                primaryController->setSecAssoc(secCntrls);

                // start controller
                for (auto& [_, pair] : self->controllers)
                {
                    pair.first->start(pair.second);
                }
                self->status = Status::Start;
                });
        });
    }
}

void NVMeSubsystem::start()
{
    // add thermal sensor for the subsystem
    std::optional<std::string> sensorName = createSensorNameFromPath(path);
    if (!sensorName)
    {
        // fail to parse sensor name from path, using name instead.
        sensorName.emplace(name);
    }

    std::vector<thresholds::Threshold> sensorThresholds;
    if (!parseThresholdsFromConfig(config, sensorThresholds))
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
        ctemp_fetch_t<NVMeBasicIntf::DriveStatus*> dataFether =
            [intf, self{std::move(shared_from_this())}](
                std::function<void(const std::error_code&,
                                   NVMeBasicIntf::DriveStatus*)>&& cb) {
            /* Potentially defer sampling the sensor sensor if it is in error */
            if (!self->ctemp->sample())
            {
                cb(std::make_error_code(std::errc::operation_canceled),
                   nullptr);
                return;
            }

            intf->getStatus(std::move(cb));
        };
        ctemp_process_t<NVMeBasicIntf::DriveStatus*> dataProcessor =
            [self{shared_from_this()}](const std::error_code& error,
                                       NVMeBasicIntf::DriveStatus* status) {
            // deferred sampling
            if (error == std::errc::operation_canceled)
            {
                return;
            }
            // The device is physically absent
            else if (error == std::errc::no_such_device)
            {
                std::cerr << "error reading ctemp from subsystem"
                          << ", reason:" << error.message() << "\n";
                self->ctemp->markFunctional(false);
                self->ctemp->markAvailable(false);
                return;
            }
            // other communication errors
            else if (error)
            {
                std::cerr << "error reading ctemp from subsystem"
                          << ", reason:" << error.message() << "\n";
                self->ctemp->incrementError();
                return;
            }

            if (status == nullptr)
            {
                std::cerr << "empty data returned by data fetcher" << std::endl;
                self->ctemp->markFunctional(false);
                return;
            }

            uint8_t flags = status->Status;
            if (((flags & NVMeBasicIntf::StatusFlags::
                              NVME_MI_BASIC_SFLGS_DRIVE_NOT_READY) != 0) ||
                ((flags & NVMeBasicIntf::StatusFlags::
                              NVME_MI_BASIC_SFLGS_DRIVE_FUNCTIONAL) == 0))
            {
                self->ctemp->markFunctional(false);
                return;
            }
            self->ctemp->updateValue(getTemperatureReading(status->Temp));
        };

        pollCtemp(ctempTimer, std::chrono::seconds(1), dataFether,
                  dataProcessor);
    }
    else if (nvmeIntf.getProtocol() == NVMeIntf::Protocol::NVMeMI)
    {
        auto intf =
            std::get<std::shared_ptr<NVMeMiIntf>>(nvmeIntf.getInferface());

        ctemp_fetch_t<nvme_mi_nvm_ss_health_status*> dataFether =
            [intf, self{std::move(shared_from_this())}](
                std::function<void(const std::error_code&,
                                   nvme_mi_nvm_ss_health_status*)>&& cb) {
            // do not poll the health status if the subsystem is intiatilzing
            if (self->status == Status::Intiatilzing)
            {
                std::cerr << "subsystem is intiatilzing, cancel the health poll"
                          << std::endl;
                cb(std::make_error_code(std::errc::operation_canceled),
                   nullptr);
                return;
            }
            intf->miSubsystemHealthStatusPoll(std::move(cb));
        };
        ctemp_process_t<nvme_mi_nvm_ss_health_status*> dataProcessor =
            [self{shared_from_this()}](const std::error_code& error,
                                       nvme_mi_nvm_ss_health_status* status) {
            if (error == std::errc::operation_canceled ||
                self->status == Status::Intiatilzing)
            {
                // on initialization, the subsystem will not update the status.
                std::cerr
                    << "subsystem is intiatilzing, do not process the status"
                    << std::endl;
                return;
            }

            if (error == std::errc::no_such_device)
            {
                std::cerr << "error reading ctemp "
                             "from subsystem"
                          << ", reason:" << error.message() << "\n";
                // stop the subsystem
                self->markFunctional(false);
                self->ctemp->markFunctional(false);
                self->ctemp->markAvailable(false);

                return;
            }
            else if (error)
            {
                std::cerr << "error reading ctemp "
                             "from subsystem"
                          << ", reason:" << error.message() << "\n";
                self->ctemp->incrementError();
                if (self->ctemp->inError())
                {
                    self->markFunctional(false);
                    self->ctemp->markFunctional(false);
                }
                return;
            }

            // Drive Functional
            bool df = status->nss & 0x20;
            if (!df)
            {
                // stop the subsystem

                self->markFunctional(false);
                self->ctemp->markFunctional(false);

                return;
            }

            // restart the subsystem
            if (self->status == Status::Stop)
            {
                self->markFunctional(true);
            }

            // TODO: update the drive interface

            self->ctemp->updateValue(getTemperatureReading(status->ctemp));
            return;
        };

        pollCtemp(ctempTimer, std::chrono::seconds(1), dataFether,
                  dataProcessor);
    }

    // start plugin
    if (plugin)
    {
        plugin->start();
    }
}
void NVMeSubsystem::stop()
{
    ctempTimer->cancel();
    if (status == Status::Start)
    {
        std::cerr << "status start" << std::endl;
        markFunctional(false);
        ctemp->markFunctional(false);
    }
    else if (status == Status::Intiatilzing)
    {
        std::cerr << "status init" << std::endl;
        ctempTimer->expires_after(std::chrono::milliseconds(100));
        ctempTimer->async_wait(
            [self{shared_from_this()}](boost::system::error_code ec) {
            if (ec)
            {
                return;
            }
            self->stop();
        });
    }
}
