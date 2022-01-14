#include <fcntl.h>
#include <unistd.h>

#include <FileHandle.hpp>

FileHandle::FileHandle(const std::filesystem::path& name,
                       std::ios_base::openmode mode) :
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    fd(open(name.c_str(), mode))
{
    if (fd <= 0)
    {
        throw std::exception();
    }
}

FileHandle::FileHandle(int fdIn) : fd(fdIn){};

FileHandle::FileHandle(FileHandle&& in)
{
    fd = in.fd;
    in.fd = -1;
}

FileHandle& FileHandle::operator=(FileHandle&& in)
{
    fd = in.fd;
    in.fd = -1;
    return *this;
}

FileHandle::~FileHandle()
{
    if (fd)
    {
        close(fd);
    }
}

int FileHandle::handle()
{
    return fd;
}
