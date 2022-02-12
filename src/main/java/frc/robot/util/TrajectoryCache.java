package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryCache {
    public enum PATHTYPE {
        PathWeaver, PathPlanner
    }

    private static HashMap<String, Trajectory> cache = new HashMap<String, Trajectory>();

    /**
     * Loads and caches a PathWeaver-based trajectory
     *
     * @param key a unique key for this trajectory
     * @param jsonPath the path to the JSON file generated from PathWeaver's "Build Paths"
     */
    public static void addPathWeaver(String key, String jsonPath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(jsonPath);
            cache.put(key, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory (PathWeaver): " + jsonPath, ex.getStackTrace());
        }
    }

    /**
     * Loads and caches a PathPlanner-based trajectory (assumes a max velocity of 1 m/s and max acceleration of 1 m/s^s)
     *
     * @param key a unique key for this trajectory
     * @param name the name of the .path file without the extension (unlike PathWeaver, only include the name, not the full path to the file)
     */
    public static void addPathPlanner(String key, String name) {
       addPathPlanner(key, name, 1, 1);
    }

    /**
     * Loads and caches a PathPlanner-based trajectory
     *
     * @param key a unique key for this trajectory
     * @param name the name of the .path file without the extension (unlike PathWeaver, only include the name, not the full path to the file)
     * @param maxVel maximum velocity (m/s)
     * @param maxAccel maximum acceleration (m/s^2)
     */
    public static void addPathPlanner(String key, String name, double maxVel, double maxAccel) {
        try {
            cache.put(key, PathPlanner.loadPath(name, maxVel, maxAccel));
        } catch (Exception ex) {
            DriverStation.reportError("Unable to open trajectory (PathPlanner): " + name, ex.getStackTrace());
        }
    }

    /**
     * Loads and caches a trajectory (currently supports PathWeaver or PathPlanner)
     *
     * @param key a unique key for this trajectory
     * @param name depending on the type of trajectory, the path to the JSON file generated from PathWeaver's "Build Paths" OR the name of the .path file without the extension (unlike PathWeaver, only include the name, not the full path to the file)
     * @param type the type of trajectory (i.e. the name of the tool used to create the path)
     */
    public static void add(String key, String name, PATHTYPE type) {
        if(type == PATHTYPE.PathWeaver)
            addPathWeaver(key, name);
        else if(type == PATHTYPE.PathPlanner)
            addPathPlanner(key, name);
    }

    /**
     * Loads and caches a PathPlanner-based trajectory
     *
     * @param key a unique key for the trajectory you want to retrieve from the cache
     * @return the trajectory as stored in the cache (will be null if no such trajectory exists in the cache)
     */
    public static Trajectory get(String key) {
        return cache.get(key);
    }

    public static void clear() {
        cache.clear();
    }
}
