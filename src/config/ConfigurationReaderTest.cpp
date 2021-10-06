#include <gtest/gtest.h>
#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include <fstream>
#include <iostream>
#include <string>


namespace robogen {

  /**
   * Check that the help is displayed
   */
  TEST(ConfigurationReaderTest, DisplaysHelp) {
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile("help");
	ASSERT_TRUE(config == NULL);
  }

  /**
   * Check that the swarm size gets parsed correctly
   */
  TEST(ConfigurationReaderTest, ParsesSwarmSize) {
    std::string swarm_size_path = "../examples/test_cases/TestParsesSwarmSize.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(swarm_size_path);

	ASSERT_TRUE(config != NULL) << "ConfigurationReader is null";
    ASSERT_EQ(config->getSwarmSize(), 3);
  }

  /**
   * Check that the Racing Scenario gets parsed correctly
   */
  TEST(ConfigurationReaderTest, ParsesRacingScenario) {
    std::string RACING_SCENARIO_PATH = "../examples/test_cases/TestParsesRacingScenario.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(RACING_SCENARIO_PATH);
	ASSERT_TRUE(config != NULL) << "ConfigurationReader is null";
    ASSERT_EQ(config->getScenario(), "racing");
  }

  /**
   * Check that the Chasing Scenario gets parsed correctly
   */
  TEST(ConfigurationReaderTest, ParsesChasingScenario) {
    std::string CHASING_SCENARIO_PATH = "../examples/test_cases/TestParsesChasingScenario.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(CHASING_SCENARIO_PATH);
	ASSERT_TRUE(config != NULL) << "ConfigurationReader is null";
    ASSERT_EQ(config->getScenario(), "chasing");
  }

  /**
   * Check that the swarm position file is being read correctly.
   */
  TEST(ConfigurationReaderTest, ReadsSwarmPosFile) {
    std::string SWARM_POS_CONFIG_PATH = "../examples/test_cases/TestReadsSwarmPosFile.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(SWARM_POS_CONFIG_PATH);
	ASSERT_TRUE(config != NULL) << "ConfigurationReader is null";
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().size(), 3);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(0).x(), 0);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(0).y(), 0);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(0).z(), 0);

    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(1).x(), 3);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(1).y(), 0);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(1).z(), 0);

    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(2).x(), 0);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(2).y(), 3);
    ASSERT_EQ(config->getSwarmPositionsConfig()->getCoordinates().at(2).z(), 0);
  }

  /**
   * Check that invalid swarm sizes are rejected
   */
  TEST(ConfigurationReaderTest, ThrowsOnBadSwarmSize) {
    std::string SWARM_POS_CONFIG_PATH = "../examples/test_cases/TestThrowsOnBadSwarmSize.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(SWARM_POS_CONFIG_PATH);
	ASSERT_TRUE(config == NULL);
  }

  /**
   * Check that the gathering zone is read correctly
   */
  TEST(ConfigurationReaderTest, ParsesGatheringZone) {
    std::string swarm_size_path = "../examples/test_cases/TestParsesGatheringZone.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(swarm_size_path);

	ASSERT_TRUE(config != NULL) << "ConfigurationReader is null";
  }

  /**
   * Check that the gathering zone position is read correctly
   */
  TEST(ConfigurationReaderTest, ParsesGatheringZonePos) {
    std::string swarm_size_path = "../examples/test_cases/TestParsesGatheringZonePos.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(swarm_size_path);

    ASSERT_EQ(config->getGatheringZonePos().x(), 42);
    ASSERT_EQ(config->getGatheringZonePos().y(), 42);
    ASSERT_EQ(config->getGatheringZonePos().z(), 42);
  }

  /**
   * Check that the gathering zone size is read correctly
   */
  TEST(ConfigurationReaderTest, ParsesGatheringZoneSize) {
    std::string swarm_size_path = "../examples/test_cases/TestParsesGatheringZoneSize.txt";
	boost::shared_ptr<RobogenConfig> config =
			ConfigurationReader::parseConfigurationFile(swarm_size_path);

    ASSERT_EQ(config->getGatheringZoneSize().x(), 42);
    ASSERT_EQ(config->getGatheringZoneSize().y(), 42);
    ASSERT_EQ(config->getGatheringZoneSize().z(), 42);
  }


} // namespace robogen
