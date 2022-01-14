#include <fcntl.h>
#include <unistd.h>

#include <FileHandle.hpp>

#include <iostream>
#include <stdexcept>

FileHandle::FileHandle(const std::filesystem::path& name,
                       std::ios_base::openmode mode) :
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    fd(open(name.c_str(), mode))
{
    if (fd <= 0)
    {
        throw std::out_of_range(name.string() + " failed to open");
    }
}

FileHandle::FileHandle(int fdIn) : fd(fdIn){};

FileHandle::FileHandle(FileHandle&& in) noexcept
{
    fd = in.fd;
    in.fd = -1;
}

FileHandle& FileHandle::operator=(FileHandle&& in) noexcept
{
    fd = in.fd;
    in.fd = -1;
    return *this;
}

FileHandle::~FileHandle()
{
    if (fd)
    {
        int r = close(fd);
        if (r < 0)
        {
            std::cerr << "Failed to close fd " << std::to_string(fd);
        }
    }
}

int FileHandle::handle()
{
    return fd;
}
