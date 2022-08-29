#include "NVMeBasic.hpp"
#include "NVMeSensor.hpp"
#include "Utils.hpp"

class NVMeSubsystem : public std::enable_shared_from_this<NVMeSubsystem>
{
  public:
    static constexpr const char* sensorType = "NVME1000";

    NVMeSubsystem(boost::asio::io_context& io,
                  sdbusplus::asio::object_server& objServer,
                  std::shared_ptr<sdbusplus::asio::connection> conn,
                  const std::string& path, const std::string& name,
                  const SensorData& configData, NVMeIntf intf);

    void start();

    void stop()
    {
        ctempTimer.cancel();
    }

  private:
    boost::asio::io_context& io;
    sdbusplus::asio::object_server& objServer;
    std::shared_ptr<sdbusplus::asio::connection> conn;
    std::string path;
    std::string name;

    NVMeIntf nvmeIntf;

    /* thermal sensor for the subsystem */
    std::optional<NVMeSensor> ctemp;
    boost::asio::steady_timer ctempTimer;

    // Function type for fetching ctemp which encaplucated in a structure of T.
    // The fetcher function take a callback as input to process the result.
    template <class T>
    using ctemp_fetcher_t =
        std::function<void(std::function<void(const std::error_code&, T)>&&)>;

    // Function type for parsing ctemp out the structure of type T.
    // The parser function will return the value of ctemp or nullopt on failure.
    template <class T>
    using ctemp_parser_t = std::function<std::optional<double>(T data)>;

    template <class T>
    void pollCtemp(const ctemp_fetcher_t<T>& dataFetcher,
                   const ctemp_parser_t<T>& dataParser);

    // implemetation details for the class.
    // It should contain only static function and using binding to the
    // NVMeSubsystem instances. The detail is defined to claim the accessibility
    // to the parent private field.
    class Detail
    {
      public:
        template <class T>
        static void pollCtemp(std::shared_ptr<NVMeSubsystem> self,
                              ctemp_fetcher_t<T> dataFetcher,
                              ctemp_parser_t<T> dataParser,
                              boost::system::error_code errorCode);
        template <class T>
        static void updateCtemp(const std::shared_ptr<NVMeSubsystem>& self,
                                ctemp_parser_t<T> dataParser,
                                ctemp_fetcher_t<T> dataFetcher,
                                boost::system::error_code errorCode, T data);
    };
};
