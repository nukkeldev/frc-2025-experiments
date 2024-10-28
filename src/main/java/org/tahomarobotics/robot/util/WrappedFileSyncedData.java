package org.tahomarobotics.robot.util;

import java.io.Serializable;
import java.util.function.Function;

/**
 * File-backed data for persistence across runs that is stored as a different type.
 *
 * @param <T> The type of data
 * @param <U> The type to be (de)serialized
 */
public class WrappedFileSyncedData<T, U extends Serializable> {
    private final FileSyncedData<U> data;

    private final Function<T, U> toBacking;
    private final Function<U, T> fromBacking;

    /**
     * @param filename    filename
     * @param defaultData data is initialized to these values when no file is found
     */
    public WrappedFileSyncedData(String filename, U defaultData, Function<T, U> toBacking, Function<U, T> fromBacking) {
        data = new FileSyncedData<>(filename, defaultData);

        this.toBacking = toBacking;
        this.fromBacking = fromBacking;
    }

    public void set(T data) {
        this.data.set(toBacking.apply(data));
    }

    public T get() {
        return fromBacking.apply(data.get());
    }
}
