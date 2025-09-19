#include "FileHandle.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/lg2.hpp>

#include <filesystem>
#include <iostream>
#include <stdexcept>

FileHandle::FileHandle(const std::filesystem::path& name,
                       std::ios_base::openmode mode) :
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
    fd(open(name.c_str(), mode))
{
    if (fd < 0)
    {
        throw std::out_of_range(name.string() + " failed to open");
    }
}

FileHandle::FileHandle(int fdIn) : fd(fdIn) {};

FileHandle::FileHandle(FileHandle&& in) noexcept : fd(in.fd)
{
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
    if (fd >= 0)
    {
        int r = close(fd);
        if (r < 0)
        {
            lg2::error("Failed to close fd: '{FD}'", "FD", fd);
        }
    }
}

int FileHandle::handle() const
{
    return fd;
}
