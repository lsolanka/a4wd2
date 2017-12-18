#include <sstream>
#include <string>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <sensor_reader/sensor_reader.hpp>
#include <sensor_reader/srf08.hpp>
#include <sensor_reader/link_interfaces.hpp>

using json = nlohmann::json;
using a4wd2::sensor_reader::sensor_reader;
namespace sensors = a4wd2::sensor_reader::sensors;

TEST(sensor_reader, test_register_sensor)
{
    json test_values = {
        {"srf08", {
            {"a", 0x70},
            {"r", 123},
            {"l", 50}
        }}
    };
    std::stringstream ss;
    ss << test_values;
    sensor_reader<std::stringstream> reader{ss};

    std::vector<sensors::srf08::data_t> data_vec;
    auto on_data = [&] (const auto& data)
        {
            data_vec.push_back(data);
        };

    reader.add_sensor<sensors::srf08>(0x70, on_data);
    reader.read_all();

    ASSERT_EQ(1, data_vec.size());
    ASSERT_EQ(test_values["srf08"]["a"].get<int>(), data_vec[0].address);
    ASSERT_EQ(test_values["srf08"]["r"].get<int>(), data_vec[0].range);
    ASSERT_EQ(test_values["srf08"]["l"].get<int>(), data_vec[0].light);
}

TEST(sensor_reader, test_invalid_json)
{
    std::stringstream ss("invalid json");
    sensor_reader<std::stringstream> reader{ss};

    std::vector<sensors::srf08::data_t> data_vec;
    auto on_data = [&] (const auto& data)
        {
            data_vec.push_back(data);
        };

    reader.add_sensor<sensors::srf08>(0x70, on_data);
    reader.read_all();

    ASSERT_EQ(0, data_vec.size());
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
