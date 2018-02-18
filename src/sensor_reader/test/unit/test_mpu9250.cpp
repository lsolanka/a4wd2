#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <sensor_reader/mpu9250.hpp>

using json = nlohmann::json;
namespace sensors = a4wd2::sensor_reader::sensors;

TEST(test_mpu9250, test_valid_data)
{
    json test_values{{"ax", -100}, {"ay", 200}, {"az", 300},

                     {"gx", -400}, {"gy", 500}, {"gz", 600},

                     {"mx", -700}, {"my", 800}, {"mz", 900}};

    sensors::mpu9250::data_t data;
    sensors::mpu9250 sensor{[&data](const auto& d) { data = d; }};
    ASSERT_TRUE(sensor.parse(test_values));
    EXPECT_EQ(test_values["ax"], data.ax);
    EXPECT_EQ(test_values["ay"], data.ay);
    EXPECT_EQ(test_values["az"], data.az);

    EXPECT_EQ(test_values["gx"], data.gx);
    EXPECT_EQ(test_values["gy"], data.gy);
    EXPECT_EQ(test_values["gz"], data.gz);

    EXPECT_EQ(test_values["mx"], data.mx);
    EXPECT_EQ(test_values["my"], data.my);
    EXPECT_EQ(test_values["mz"], data.mz);
}

TEST(test_mpu9250, test_missing_values)
{
    sensors::mpu9250::data_t data;
    sensors::mpu9250 sensor{[&data](const auto& d) { data = d; }};

    ASSERT_THROW(sensor.parse(json{}), json::exception);
    ASSERT_THROW(sensor.parse(json{{"ax", -100}, {"ay", 200}, {"az", 300}}),
                 json::exception);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
