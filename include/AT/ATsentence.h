#include <string>
#include <sstream>
#include <vector>

struct AtType
{
    std::string command;

    std::vector<std::string> fields;
};

class ATsentence
{
public:
    ATsentence();
    std::string encode(const AtType &at_sentance);
    AtType decode(const std::string &);

private:
};
