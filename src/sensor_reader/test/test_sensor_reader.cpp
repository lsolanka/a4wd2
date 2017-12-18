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
    std::vector<json> test_values;
    test_values.push_back({
        {"srf08", {
            {"a", 0x70},
            {"r", 123},
            {"l", 50}
        }}
    });
    test_values.push_back({
        {"srf08", {
            {"a", 0x71},
            {"r", 10},
            {"l", 20}
        }}
    });
    test_values.push_back({ // should be ignored
        {"other", {
            {"a", 0x71},
            {"r", 10},
            {"l", 20}
        }}
    });

    // Fill in the string passed to reader
    std::stringstream ss;
    for (auto& val : test_values)
    {
        ss << val << '\n';
    }
    sensor_reader reader{ss};

    std::vector<sensors::srf08::data_t> data_vec_0x70;
    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            data_vec_0x70.push_back(data);
        }
    );

    std::vector<sensors::srf08::data_t> data_vec_0x71;
    reader.add_sensor<sensors::srf08>(0x71, [&] (const auto& data)
        {
            data_vec_0x71.push_back(data);
        }
    );
    reader.read_all();

    ASSERT_EQ(1, data_vec_0x70.size());
    ASSERT_EQ(test_values[0]["srf08"]["a"].get<int>(), data_vec_0x70[0].address);
    ASSERT_EQ(test_values[0]["srf08"]["r"].get<int>(), data_vec_0x70[0].range);
    ASSERT_EQ(test_values[0]["srf08"]["l"].get<int>(), data_vec_0x70[0].light);

    ASSERT_EQ(1, data_vec_0x71.size());
    ASSERT_EQ(test_values[1]["srf08"]["a"].get<int>(), data_vec_0x71[0].address);
    ASSERT_EQ(test_values[1]["srf08"]["r"].get<int>(), data_vec_0x71[0].range);
    ASSERT_EQ(test_values[1]["srf08"]["l"].get<int>(), data_vec_0x71[0].light);
}

TEST(sensor_reader, two_sensors_same_address)
{
    json test_values = {
        {"srf08", {
            {"a", 0x70},
            {"r", 123},
            {"l", 50}
        }}
    };

    // Fill in the string passed to reader
    std::stringstream ss;
    ss << test_values << '\n';
    sensor_reader reader{ss};

    std::vector<sensors::srf08::data_t> data_vec;
    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            data_vec.push_back(data);
        }
    );

    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            data_vec.push_back(data);
        }
    );
    reader.read_all();

    ASSERT_EQ(2, data_vec.size());
    ASSERT_EQ(test_values["srf08"]["a"].get<int>(), data_vec[0].address);
    ASSERT_EQ(test_values["srf08"]["r"].get<int>(), data_vec[0].range);
    ASSERT_EQ(test_values["srf08"]["l"].get<int>(), data_vec[0].light);

    ASSERT_EQ(test_values["srf08"]["a"].get<int>(), data_vec[1].address);
    ASSERT_EQ(test_values["srf08"]["r"].get<int>(), data_vec[1].range);
    ASSERT_EQ(test_values["srf08"]["l"].get<int>(), data_vec[1].light);
}

TEST(sensor_reader, top_level_no_object)
{
    // We provide an array
    json test_values = json::array({10, 20, json({ {"blabla", 10} })});

    std::stringstream ss;
    ss << test_values << '\n';
    sensor_reader reader{ss};

    std::vector<sensors::srf08::data_t> data_vec;
    reader.add_sensor<sensors::srf08>(0x70, [&] (const auto& data)
        {
            data_vec.push_back(data);
        }
    );

    reader.read_all();

    ASSERT_TRUE(data_vec.empty());
}

TEST(sensor_reader, test_invalid_json)
{
    std::stringstream ss("invalid json");
    sensor_reader reader{ss};

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
