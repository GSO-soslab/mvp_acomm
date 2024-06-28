
#include "ATsentence.h"
#include "string"

ATsentence::ATsentence()
{
}

std::string ATsentence::encode(const AtType &at_sentence)
{
    std::stringstream message;

    message << "+++AT" << at_sentence.command;

    for (int i = 0; i < at_sentence.fields.size(); i++)
    {
        message << "," << at_sentence.fields.at(i);
    }

    return message.str();
}

AtType ATsentence::decode(const std::string &raw)
{

    if (raw.empty())
        throw "Bad decode";

    // We need to determine what type of message it is. It will have ? on a read, ! on a write, and neither on usbl

    std::string::size_type read_position = raw.find_first_of('?');
    std::string::size_type write_position = raw.find_first_of('!');

    std::string::size_type a_position = raw.find_first_of('A');
    std::string::size_type colon_position = raw.find_first_of(':');
    std::string::size_type delimeter_index = raw.find_first_of(',');

    AtType at_sentence;

    if (read_position != std::string::npos)
    {
        // its a read message
    }
    else if (write_position != std::string::npos)
    {
        // Calculate the read length.
        std::string::size_type read_length;

        // Following comma found, set length using position of next comma.
        read_length = colon_position - a_position;
        // its a write message
        at_sentence.command = raw.substr(a_position, read_length);

        std::string::size_type next_delimiter_index = raw.find_first_of(':', colon_position + 1);

        at_sentence.fields.push_back(raw.substr(next_delimiter_index + 1));

        return at_sentence;
    }

    std::string::size_type next_delimiter_index = raw.find_first_of(':', colon_position + 1);

    // Calculate the read length.
    std::string::size_type read_length;

    read_length = delimeter_index - next_delimiter_index;

    at_sentence.command = raw.substr(next_delimiter_index + 1, read_length-1);

    while (delimeter_index != std::string::npos)
    {
        // Delimeter is pointing to the comma at the start of this field.

        // Find the index of the next comma.
        std::string::size_type next_delimiter_index = raw.find_first_of(',', delimeter_index + 1);

        // Calculate the read length.
        std::string::size_type read_length;

        // Following comma found, set length using position of next comma.
        read_length = next_delimiter_index - delimeter_index - 1;

        // Pull substring into field.
        at_sentence.fields.push_back(raw.substr(delimeter_index + 1, read_length));

        // Update delimeter index.
        delimeter_index = next_delimiter_index;
    }

    return at_sentence;
}
