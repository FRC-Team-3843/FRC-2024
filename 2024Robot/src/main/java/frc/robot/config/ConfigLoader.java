package frc.robot.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

public final class ConfigLoader {
  private static final String CONFIG_FILE_NAME = "robot-config.json";
  private static RobotConfig cachedConfig = null;

  private ConfigLoader() {}

  public static synchronized RobotConfig load() {
    if (cachedConfig != null) {
      return cachedConfig;
    }

    File deployDirectory = Filesystem.getDeployDirectory();
    File configFile = new File(deployDirectory, CONFIG_FILE_NAME);
    ObjectMapper mapper = new ObjectMapper();

    try {
      cachedConfig = mapper.readValue(configFile, RobotConfig.class);
    } catch (IOException e) {
      System.err.println("CRITICAL ERROR: Could not load robot configuration from " + configFile.getAbsolutePath());
      e.printStackTrace();
      throw new RuntimeException("Failed to load robot configuration", e);
    }

    return cachedConfig;
  }
}
