#include <stdexcept>
hashtag#include <string>

namespace wakeb_exceptions
{

class TaskException : public std::runtime_error
{
public:
 explicit TaskException(const std::string& description) : std::runtime_error(description)
 {
 }
};

class InvalidTaskStartPosition : public TaskException
{
public:
 explicit InvalidTaskStartPosition(const std::string& description) : TaskException(description)
 {
 }
};

// task body parsing
class InvalidTaskJsonDetails : public TaskException
{
public:
 explicit InvalidTaskJsonDetails(const std::string& description) : TaskException(description)
 {
 }
};
class InvalidTaskJsonElements : public TaskException
{
public:
 explicit InvalidTaskJsonElements(const std::string& description) : TaskException(description)
 {
 }
};

} // namespace wakeb_exceptions