# Logger System Design & Usage

This document explains the design, limitations, and usage of the Skyward logging system. It is intended for developers who need to log structured data efficiently and reliably, and for those who wish to extend or maintain the logger.

---

## Overview

The logger is a high-performance, buffered logging system designed for embedded environments. It serializes user-defined types to binary log files, using [socrate](https://git.skywarder.eu/avn/swd/socrate) for reflection and serialization. The logger supports concurrent logging from multiple threads, minimizes blocking, and ensures data integrity even in the event of power loss (when stopped properly).

---

## Limitations

- **Trivially Copyable Types:**  
  All types to be logged must be trivially copyable. This means no pointers to dynamic memory, virtual methods, or non-trivial destructors.
- **Reflection Required:**  
  Every struct or class to be logged must implement a `reflect()` method compatible with socrate's reflection system.
- **Unique Type Names:**  
  Each logged type must have a unique name. Namespaces are not stored in the log, so type names must not collide.
- **Buffer Size Constraints:**  
  The maximum size of a single log record and its mapping is limited (see `Logger.h` for `maxRecordSize` and `maxMappingSize`). Large types may require increasing these limits.

---

## How the magic works

1. **Reflection & Serialization:**  
   When you log a struct, the logger uses socrate to inspect its fields and serialize its contents into a binary format.
2. **Type Mapping:**  
   The first time a type is logged in a new file, the logger creates a *mapping* string describing the structure of the type. This mapping is prefixed by a special marker (`!`) in the log file. This string is always written to file before the actual data.
3. **Concurrent Logging:**  
   Logging is non-blocking for the caller. Data is placed in a queue and packed into buffers by a background thread, then written to disk by another thread.
4. **Decoding:**  
   When reading logs, the mapping strings allow the decoder to reconstruct the structure of each type, enabling type-safe deserialization.

---

## Example: making a struct loggable

To log a struct, you must define a `reflect()` method using socrate macros:

```cpp
struct VeryInner
{
    int fifth;
};

struct Inner
{
    VeryInner inner;
};

struct Foo
{
    uint32_t first;
    float32_t second;
    int third[4];
    Inner inner;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Foo, FIELD_DEF(first)
                                   FIELD_DEF(second)
                                   FIELD_DEF(third)
                                   FIELD_DEF3(inner, inner, fifth));
    }
};
```

- Use `FIELD_DEF(name)` for direct fields.
- Use `FIELD_DEF2()` or `FIELD_DEF3()` for nested fields (see socrate documentation for more).
- If your struct extends another struct, use `EXTEND_DEF(name)`.
- Ensure all fields are compatible with socrate serialization.
- For more examples check the socrate library
---

## Logging data

To log an instance of your struct:

```cpp
Foo foo{...};
Logger::getInstance().log(foo);
```

- The logger will automatically handle type mapping and serialization.
- If the logger is not started, the call is ignored or dropped (see return value).

---

## Log file format

- **Mappings:**  
  Each new type is described by a mapping string, marked by the character `!` at the start. The mapping includes the type name, number of fields, field names, and type IDs (each type is given an ID, see `Logger.h` for more info).
- **Data:**  
  Each logged instance is serialized with its type name and field data.

---

## Error handling

- If buffers are full, data may be dropped (see `LoggerResult::Dropped`).
- If the logger is stopped, data is ignored.
- If a type is too large, logging fails and a warning is printed.

---


## Extending the logger

- To support new types, ensure they are trivially copyable and provide a `reflect()` method.
- For custom serialization, specialize `socrate::userde::Serde<T>`.

---

## References

- [socrate reflection library](https://git.skywarder.eu/avn/swd/socrate)
- See `Logger.h` and `Logger.cpp` for implementation details.

---
