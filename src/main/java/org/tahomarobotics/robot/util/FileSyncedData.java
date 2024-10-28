/**
 * Copyright 2023 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * <p>
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * <p>
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions
 * of the Software.
 * <p>
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.*;
import java.util.Arrays;

/**
 * File-backed data for persistence across runs.
 *
 * @param <T> The type of data
 */
public class FileSyncedData<T extends Serializable> {
    private final static Logger logger = LoggerFactory.getLogger(FileSyncedData.class);

    private static final File HOME_DIR = Filesystem.getOperatingDirectory();

    private final File file;
    /**
     * A singleton array of the data.
     * Lets us leverage Arrays.deepToString to properly print the data if it is an Array.
     */
    private T[] data;

    /**
     * @param filename    filename
     * @param defaultData data is initialized to these values when no file is found
     */
    public FileSyncedData(String filename, T defaultData) {
        this.data = castData(defaultData);
        this.file = new File(HOME_DIR, filename);
        readCalibrationFile();
    }

    @SuppressWarnings("unchecked")
    private void readCalibrationFile() {
        try (ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(file))) {
            data = castData((T) inputStream.readObject());
            logger.info("Successfully read calibration data <{}> -> {}", file.getAbsolutePath(), formatData());
        } catch (Exception e) {
            logger.error("Failed to read calibration data <{}>", file.getAbsolutePath());
        }
    }

    private void writeCalibrationFile(T[] data) {
        try (ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(file))) {
            outputStream.writeObject(data[0]);
            logger.warn("Wrote new calibration data <{}> -> {}", file.getAbsolutePath(), formatData());
        } catch (Exception e) {
            logger.error("Failed to write calibration data <{}>: {}", file.getAbsolutePath(), e);
        }
    }

    /**
     * Returns the calibration data read from file if successful otherwise the default data
     */
    public T get() {
        return data[0];
    }

    /**
     * Applies a new set of data to the configuration file
     */
    public void set(T data) {
        this.data = castData(data);
        writeCalibrationFile(this.data);
    }

    @SuppressWarnings("unchecked")
    private T[] castData(T data) {
        return (T[]) new Serializable[]{data};
    }

    private String formatData() {
        String tmp = Arrays.deepToString(data).substring(1);
        return tmp.substring(0, tmp.length() - 1);
    }
}
