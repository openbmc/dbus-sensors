#pragma once

#include <filesystem>
#include <ios>

// An RAII compliant object for holding a posix file descriptor
class FileHandle
{
  private:
    int fd;

  public:
    FileHandle() = delete;
    FileHandle(const FileHandle&) = delete;
    FileHandle& operator=(const FileHandle&) = delete;
    FileHandle(FileHandle&& /*in*/) noexcept;
    FileHandle& operator=(FileHandle&& /*in*/) noexcept;

    explicit FileHandle(const std::filesystem::path& name,
                        std::ios_base::openmode mode = std::ios_base::in |
                                                       std::ios_base::out);

    explicit FileHandle(int fd);

    ~FileHandle();
    int handle() const;
};
