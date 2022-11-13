#include "RedfishSensor.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>

static constexpr bool debug = false;

// Dumps all content received, dangerous (no escaping), only if debug true
static constexpr bool dumpContent = false;

// FUTURE: This parameter could come in from configuration
static constexpr double networkTimeout = 30;

// Arbitrary limit on network request and reply size, in bytes
static constexpr int sizeLimit = 65536;

// This one goes up to 11
static constexpr int httpVersion = 11;

// FUTURE: Change to std::string type when allowed by compiler
static constexpr auto userAgent = "RedfishSensor/1.0";
static constexpr auto redfishRoot = "/redfish/v1";

// This class acts as an oracle, taking a string from caller, and eventually
// calling the callback, supplying another string as an answer. The caller
// does not need to know any details of what goes on behind the scenes.
class RedfishConnection : public std::enable_shared_from_this<RedfishConnection>
{
  public:
    RedfishConnection(std::shared_ptr<boost::asio::io_service> io,
                      const std::string& host, const std::string& protocol) :
        ioContext(io),
        mockTimer(*io), netResolver(boost::asio::make_strand(*io)),
        netStream(boost::asio::make_strand(*io)), serverHost(host),
        serverProtocol(protocol)
    {
        // No body necessary
    }

    // Call this before submitTransaction to mock that transaction
    void enableMock(int msDelay, int mockErrorCode,
                    const std::string& mockResponse);

    void submitTransaction(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);

  private:
    bool mockEnabled = false;
    int amockTime = 0;
    int mockError = 0;
    std::string mockReply;

    bool isResolved = false;
    bool isConnected = false;

    std::shared_ptr<boost::asio::io_service> ioContext;
    boost::asio::steady_timer mockTimer;

    // Stuff to feed the Beast with
    boost::asio::ip::tcp::resolver netResolver;
    boost::beast::tcp_stream netStream;
    boost::beast::flat_buffer netBuffer;
    boost::beast::http::request<boost::beast::http::empty_body> httpReq;
    boost::beast::http::response<boost::beast::http::string_body> httpResp;
    std::optional<
        boost::beast::http::request_serializer<boost::beast::http::empty_body>>
        reqSerializer;
    std::optional<
        boost::beast::http::response_parser<boost::beast::http::string_body>>
        respParser;

    // For resolution
    // FUTURE: Add support for SSL (if https), TCP port, user, password
    std::string serverHost;
    std::string serverProtocol;
    boost::asio::ip::tcp::endpoint serverEndpoint;

    uint64_t bytesRead = 0;
    uint64_t bytesWrite = 0;

    int transactionsAttempted = 0;
    int transactionsCompleted = 0;

    void mockTransaction(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);

    void submitResolution(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);
    void submitConnection(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);
    void submitQuery(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);
    void submitReply(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback);

    void handleResolution(
        const std::string& request,
        std::function<void(const std::string& response, int code)> callback,
        const boost::system::error_code& ec,
        const boost::asio::ip::tcp::resolver::results_type& results);
};

void RedfishConnection::enableMock(int msDelay, int mockErrorCode,
                                   const std::string& mockResponse)
{
    mockEnabled = true;
    amockTime = msDelay;
    mockError = mockErrorCode;
    mockReply = mockResponse;
}

void RedfishConnection::mockTransaction(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    auto weakThis = weak_from_this();

    // Each mock is a one-shot
    mockEnabled = false;

    mockTimer.expires_after(std::chrono::milliseconds(amockTime));

    mockTimer.async_wait(
        [weakThis, callback](const boost::system::error_code& ec) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            std::cerr << "Timer callback error: Object disappeared\n";
            return;
        }
        if (ec)
        {
            std::cerr << "Timer callback error: " << ec.message();
            return;
        }

        callback(lockThis->mockReply, lockThis->mockError);

        if constexpr (debug)
        {
            std::cerr << "Mock transaction completed: "
                      << lockThis->mockReply.size() << " bytes, "
                      << lockThis->mockError << " code, " << lockThis->amockTime
                      << " ms\n";
            if constexpr (dumpContent)
            {
                std::cerr << lockThis->mockReply << "\n";
            }
        }
    });

    if constexpr (debug)
    {
        std::cerr << "Mock transaction submitted: " << request << "\n";
    }
}

void RedfishConnection::submitTransaction(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    ++transactionsAttempted;

    // Take the easy way out, if we can
    if (mockEnabled)
    {
        mockTransaction(request, callback);
        return;
    }

    // Callbacks will chain: Resolve -> Connect -> Query -> Reply
    if (!isResolved)
    {
        submitResolution(request, callback);
        return;
    }

    // Callbacks will chain: Connect -> Query -> Reply
    if (!isConnected)
    {
        submitConnection(request, callback);
        return;
    }

    // Things look good from before, fastest path: Query -> Reply
    submitQuery(request, callback);
    return;
}

// Handles both the blocking and async versions of the resolver
void RedfishConnection::handleResolution(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback,
    const boost::system::error_code& ec,
    const boost::asio::ip::tcp::resolver::results_type& results)
{
    isResolved = false;

    if (ec)
    {
        std::cerr << "Network resolution failure: " << ec.message() << "\n";

        // Give the user the bad news
        callback("", ec.value());
        return;
    }

    for (const auto& result : results)
    {
        if constexpr (debug)
        {
            std::cerr << "Network resolution endpoint: " << result.endpoint()
                      << "\n";
        }

        // Accept the first endpoint in the results list
        // FUTURE: Perhaps handle multiple endpoints in the future
        if (!isResolved)
        {
            isResolved = true;
            serverEndpoint = result.endpoint();
        }
    }

    if (!isResolved)
    {
        std::cerr << "Network resolution failure: No results\n";

        // Give the user the bad news
        callback("", ec.value());
        return;
    }

    // After resolution, proceed to connection
    submitConnection(request, callback);

    if constexpr (debug)
    {
        std::cerr << "Network resolution success: host " << serverHost
                  << ", endpoint " << serverEndpoint << "\n";
    }
}

// FUTURE: Allow https and/or custom port number
void RedfishConnection::submitResolution(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    // Boost limitation, async_resolve() forcefully creates a new thread
    // Workaround for the resulting exception abort if threads disabled
#ifdef BOOST_ASIO_DISABLE_THREADS
    if constexpr (debug)
    {
        std::cerr << "Network attempting blocking resolution: host "
                  << serverHost << ", protocol " << serverProtocol << "\n";
    }

    boost::system::error_code ec;
    boost::asio::ip::tcp::resolver::results_type results =
        netResolver.resolve(serverHost, serverProtocol, ec);
    handleResolution(request, callback, ec, results);

    return;
#endif

    auto weakThis = weak_from_this();

    // FUTURE: Use another timer, because resolver does not have a timeout
    netResolver.async_resolve(
        serverHost, serverProtocol,
        [weakThis, request,
         callback](const boost::system::error_code& ec,
                   boost::asio::ip::tcp::resolver::results_type results) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            std::cerr << "Network resolution failure: Object has disappeared\n";
            return;
        }
        lockThis->handleResolution(request, callback, ec, results);
        });

    if constexpr (debug)
    {
        std::cerr << "Network attempting threaded resolution: host "
                  << serverHost << ", protocol " << serverProtocol << "\n";
    }
}

void RedfishConnection::submitConnection(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    auto weakThis = weak_from_this();

    netStream.expires_after(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(networkTimeout)));

    netStream.async_connect(serverEndpoint, [weakThis, request, callback](
                                                boost::beast::error_code ec) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            std::cerr << "Network connection failure: Object has disappeared\n";
            return;
        }

        if (ec)
        {
            std::cerr << "Network connection failure: " << ec.message() << "\n";

            // FUTURE: Back all the way out, do resolve again if connect fails
            lockThis->isConnected = false;

            // Give the user the bad news
            callback("", ec.value());
            return;
        }

        // Good connection
        lockThis->isConnected = true;

        // After connection, proceed to query
        lockThis->submitQuery(request, callback);

        if constexpr (debug)
        {
            std::cerr << "Network connection success\n";
        }
    });

    if constexpr (debug)
    {
        std::cerr << "Network attempting connection: " << serverEndpoint
                  << "\n";
    }
}

void RedfishConnection::submitQuery(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    auto weakThis = weak_from_this();

    // FUTURE: Support HTTP username/password, if given, and SSL
    httpReq.method(boost::beast::http::verb::get);
    httpReq.target(request);
    httpReq.version(httpVersion);
    httpReq.keep_alive(true);
    httpReq.set(boost::beast::http::field::host, serverHost);
    httpReq.set(boost::beast::http::field::user_agent, userAgent);

    // The parser must live through async, but reconstructed before each message
    reqSerializer.reset();
    reqSerializer.emplace(httpReq);

    reqSerializer->limit(sizeLimit);

    netStream.expires_after(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(networkTimeout)));

    boost::beast::http::async_write(
        netStream, *reqSerializer,
        [weakThis, request, callback](const boost::beast::error_code ec,
                                      std::size_t bytesTransferred) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            std::cerr << "Network query failure: Object has disappeared\n";
            return;
        }

        if (ec)
        {
            std::cerr << "Network query failure: " << ec.message() << "\n";

            // Connection is unusable after error, tear it down
            lockThis->netStream.close();
            lockThis->isConnected = false;

            // Give the user the bad news
            callback("", ec.value());
            return;
        }

        lockThis->bytesWrite += bytesTransferred;

        // After query, proceed to reply
        lockThis->submitReply(request, callback);

        if constexpr (debug)
        {
            std::cerr << "Network query success: " << bytesTransferred
                      << " bytes transferred\n";
        }
        });

    if constexpr (debug)
    {
        std::cerr << "Network attempting query: " << request << "\n";
    }
}

void RedfishConnection::submitReply(
    const std::string& request,
    std::function<void(const std::string& response, int code)> callback)
{
    auto weakThis = weak_from_this();

    // The parser must live through async, but reconstructed before each message
    respParser.reset();
    respParser.emplace();

    respParser->header_limit(sizeLimit);
    respParser->body_limit(sizeLimit);

    netStream.expires_after(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(networkTimeout)));

    boost::beast::http::async_read(
        netStream, netBuffer, *respParser,
        [weakThis, request, callback](const boost::beast::error_code ec,
                                      std::size_t bytesTransferred) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            std::cerr << "Network reply failure: Object has disappeared\n";
            return;
        }

        if (ec)
        {
            std::cerr << "Network reply failure: " << ec.message() << "\n";

            // Connection is unusable after error, tear it down
            lockThis->netStream.close();
            lockThis->isConnected = false;

            // Give the user the bad news
            callback("", ec.value());
            return;
        }

        lockThis->httpResp = lockThis->respParser->release();

        lockThis->bytesRead += bytesTransferred;
        ++(lockThis->transactionsCompleted);

        int resultCode = lockThis->httpResp.result_int();
        if (lockThis->httpResp.result() != boost::beast::http::status::ok)
        {
            // No return here, this is only a warning, not an error
            std::cerr << "Network reply code: " << resultCode << " "
                      << lockThis->httpResp.result() << "\n";
        }

        // Finally some good news
        callback(lockThis->httpResp.body(), resultCode);

        if constexpr (debug)
        {
            std::cerr << "Network reply success: " << bytesTransferred
                      << " bytes transferred\n";
        }
        });

    if constexpr (debug)
    {
        std::cerr << "Network attempting reply\n";
    }
}

void RedfishTransaction::handleCallback(const std::string& response, int code)
{
    std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now();

    auto msSince =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - timeStarted)
            .count();

    if constexpr (debug)
    {
        std::cerr << "REPLY: " << response.size() << " bytes, " << code
                  << " code, " << msSince << " ms\n";

        if constexpr (dumpContent)
        {
            std::cerr << response << "\n";
        }
    }

    RedfishCompletion reply;

    reply.queryRequest = queryRequest;
    reply.queryResponse = response;
    reply.msElapsed = msSince;
    reply.errorCode = code;

    // Turn response string into JSON early, saving caller some work
    // Pass 3rd argument to avoid throwing exceptions
    reply.jsonResponse = nlohmann::json::parse(response, nullptr, false);
    if (!(reply.jsonResponse.is_object()))
    {
        // No return here, this is a warning, not a fatal error
        std::cerr << "Transaction returned content that is not a JSON object\n";
    }

    // Tell the caller what just happened
    callbackResponse(reply);

    if constexpr (debug)
    {
        std::cerr << "Reply callback called\n";
    }
}

void RedfishTransaction::submitRequest(
    std::shared_ptr<RedfishConnection> connection)
{
    if constexpr (debug)
    {
        std::cerr << "QUERY: " << queryRequest << "\n";
    }

    auto weakThis = weak_from_this();

    timeStarted = std::chrono::steady_clock::now();

    connection->submitTransaction(
        queryRequest, [weakThis](const std::string& response, int code) {
            auto lockThis = weakThis.lock();
            if (!lockThis)
            {
                std::cerr
                    << "Transaction callback ignored: Object has disappeared\n";
                return;
            }

            lockThis->handleCallback(response, code);
        });

    if constexpr (debug)
    {
        std::cerr << "Query submitted\n";
    }
}

void RedfishServer::startNetworking()
{
    if (networkConnection)
    {
        std::cerr << "Internal error: Networking already started\n";
        return;
    }

    if (host.empty())
    {
        std::cerr << "Internal error: Networking host is empty\n";
        return;
    }

    // FUTURE: Allow https (SSL), custom TCP port, HTTP user/password
    if (protocol.empty())
    {
        protocol = "http";
    }

    networkConnection =
        make_shared<RedfishConnection>(ioContext, host, protocol);
}

// To keep the flags and audits correct, wrap your transactions in these
bool RedfishServer::sendTransaction(const RedfishTransaction& transaction)
{
    if (networkBusy)
    {
        std::cerr << "Internal error: Overlapping transactions attempted\n";
        return false;
    }

    if (!networkConnection)
    {
        std::cerr << "Internal error: Networking not successfully started\n";
        return false;
    }

    ++transactionsStarted;

    networkBusy = true;

    // Retain ownership of transaction throughout the async
    activeTransaction.reset();
    activeTransaction = std::make_shared<RedfishTransaction>(transaction);

    activeTransaction->submitRequest(networkConnection);

    if constexpr (debug)
    {
        std::cerr << "Sent transaction: " << activeTransaction->queryRequest
                  << "\n";
    }

    return true;
}

bool RedfishServer::doneTransaction(const RedfishCompletion& completion)
{
    networkBusy = false;

    // Transaction is no longer active
    activeTransaction.reset();

    bool success = false;

    // Success is defined by having a valid JSON object and HTTP success
    if (completion.jsonResponse.is_object())
    {
        if (boost::beast::http::int_to_status(completion.errorCode) ==
            boost::beast::http::status::ok)
        {
            success = true;
        }
    }

    msNetworkTime += completion.msElapsed;

    if (success)
    {
        ++transactionsSuccess;
    }
    else
    {
        ++transactionsFailure;
    }

    if constexpr (debug)
    {
        std::cerr << "Done transaction: " << (success ? "Success" : "Failure")
                  << ", " << completion.msElapsed << " ms\n";
    }

    return success;
}

// FUTURE: These query functions are repetitive, consider a template
void RedfishServer::queryRoot()
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = redfishRoot;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromRoot(completion.jsonResponse))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::queryTelemetry()
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = pathTelemetry;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromTelemetry(completion.jsonResponse))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::queryMetricCollection()
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = pathMetrics;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromMetricCollection(completion.jsonResponse))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::queryMetricReport(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromMetricReport(completion.jsonResponse))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::queryChassisCollection()
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = pathChassis;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromChassisCollection(completion.jsonResponse))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::queryChassisCandidate(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse =
        [weakThis, path](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromChassisCandidate(completion.jsonResponse,
                                                   path))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::querySensorCollection(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse =
        [weakThis, path](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromSensorCollection(completion.jsonResponse,
                                                   path))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::querySensorCandidate(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse =
        [weakThis, path](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->fillFromSensorCandidate(completion.jsonResponse,
                                                  path))
            {
                lockThis->advanceDiscovery();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::collectMetricReport(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->acceptMetricReport(completion.jsonResponse,
                                             lockThis->timeReading))
            {
                lockThis->advanceReading();
            }
        }
    };

    sendTransaction(trans);
}

void RedfishServer::collectSensorReading(const std::string& path)
{
    auto weakThis = weak_from_this();

    RedfishTransaction trans;

    trans.queryRequest = path;
    trans.callbackResponse = [weakThis](const RedfishCompletion& completion) {
        auto lockThis = weakThis.lock();
        if (!lockThis)
        {
            return;
        }
        if (lockThis->doneTransaction(completion))
        {
            if (lockThis->acceptSensorReading(completion.jsonResponse,
                                              lockThis->timeReading))
            {
                lockThis->advanceReading();
            }
        }
    };

    sendTransaction(trans);
}
