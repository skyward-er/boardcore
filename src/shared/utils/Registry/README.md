# Skyward Registry

### Goal and purpose

Skyward Registry is a Skyward project to develop a configuration saving
mechanism into persistent (flash memory) and non persistent memories (RAM).

Its purpose is to save the configurations for the rocket and retrieve them after
a reboot or momentary CPU issues and transient events.

The need of such sw development has its roots in some events in which
the reboot of the rocket, and therefore its configuration, lent to a misconfiguration, taking defaults values. This caused the propulsion chamber's valves
to open with wrong timing and therefore causing a misfire.
Therefore, it is of the utmost importance to save the configuration and avoid default and hardcoded values.

## The front-end

### Invocation examples
In this case we take as example a ficticius configuration entry (NAME which has as value datatype a float)

Type-unsafe interface methods:

#### setConfigurationUnsafe
```cpp
float value = 1.3;

if(frontEnd.setUnsafe(ConfigurationEnum::NAME_CONF, value))
 { /*! correctly set */
}

if(frontEnd.setUnsafe(NAME(value))) {
    /*! correctly set */
    }
```
#### getConfigurationUnsafe
```cpp
float value;

float default = 1.35;

if(frontEnd.getConfigurationUnsafe(ConfigurationEnum::NAME_CONF, &value)) {
    /*! Getted the value */
    }

value = frontEnd.getOrSetDefaultConfigurationUnsafe(ConfigurationEnum::NAME_CONF, default);
```

#### getConfigurationOrDefaultUnsafe
```cpp
float ignitionTime, ignitionDefault = 2.0;
ignitionTime = frontEnd.getConfigurationOrDefaultUnsafe(ignitionDefault);
```

Type-safe interface methods:

#### setConfiguration
```cpp
Ignition ignitionTime(2.0);
frontEnd.setConfiguration(ignitionTime);
```
#### getConfiguration
```cpp
Ignition ignitionTime;
if(!frontEnd.getConfiguration(&ignitionTime)){
    /*! Error getting the configuration value */
}
```

#### getConfigurationOrDefault
```cpp
Ignition ignitionTime, ignitionDefault(2.0);
ignitionTime = frontEnd.getConfigurationOrDefault(ignitionDefault);
```

#### arm
```cpp
frontEnd.arm()
```

#### disarm
```cpp
frontEnd.disarm()
```

#### getConfiguredEntries
```cpp
std::unordered_set<ConfigurationEnum> configuratedIndex;
configuratedIndex = frontEnd.getConfiguredEntries();
```

#### isConfigured
```cpp
if(frontEnd.isConfigured())
{
    /*! The front end registry configuration has a configuration set */
}
```

#### isConfigured
```cpp
if(frontEnd.isEntryConfigured(ConfigurationEnum::NAME_CONF))
{
    /*! The front end configuration has such entry */
}
```

#### isConfigurationEmpty
```cpp
if(frontEnd.isConfigurationEmpty())
{
    /*! The front end configuration is empty */
}
```

#### isConfigurationCorrupted
```cpp
if(frontEnd.isConfigurationCorrupted())
{
    /*! The front end configuration is corrupted */
}
```

### How to add new structs
The correct flow to add new types/configuration entries is:

- **TypeStructure.h**: If not exist, create a new struct for the type that we will use for the configuration entry value

- **RegistryStructures.h**: If not exists, add the type to the TypeUnion struct. If not exists, add the struct for wrapping the struct above and make the methods to set/get the correct type from/to the union type.
At last, create the final data structures for the specific configuration entry.

- **RegistryFrontend.cpp**: Remember to modify the private methods for the unsafe methods such that there are methods for get/set the unionType for the new data type.


### Goals

| Goal |  Description  |
|:-----|:--------:|
| G1 | The configuration will be saved in memory |
|G2 | It will be possible to set the value of a particular configuration entry|
|G3 | It will be possible to get the value of a particular configuration entry |
|G4 | It will be possible to inspect the configured entries |
|G5 | More data types can be used as value for the different configurations entries |
|G6 | There will be a protection against changes to the configuration during flight |
|G7 | No dynamic allocations will be executed during the flight |
|G8 | The registry will offer a persistent configuration saving |
|G9 | It will be possible to verify the integrity of the configuration|
|G10 | It will be possible to explore the current configuration |
|G11 | Type safeness and thread safeness is guaranteed |

### Assumptions
The front-end, FE, considers some important assumptions about the usage of such sw component.
This assumptions are about the method call, the order of operations and other assumptions on the usage.
Such assumptions considers the actual necessities for Skyward Registry and might not be true in future.

| Assumption |  Description  |
|:-----|:--------:|
|A1 | The FE is constructed and instantiated once, a single time|
|A2 | The FE does saves and retrieves a single configuration from the registry |
|A3 | The FE is called before the flight for instantiation and registry check |
|A4 | The FE is correctly armed before the flight |
|A5 | The FE disarming is not used during flight |
|A6 | The caller considers the return of the get and set methods|
|A7 | Configuration entries does not have pointers as datatype |

### Requirements

| Requirements |  Description  |
|:-----|:--------:|
|R1 | The interface must allow setting a value for the specific configuration entries |
|R2 | The interface must allow getting a specified value for a   configuration entries |
|R3 | The interface must allow the correct data types for the   specifics entries |
|R4 | The interface must not allocate memory during the  flight phase |
|R5 | The interface must not change the configuration entries during flight phase |
|R6 | The interface does save the configuration entries using the back-end components |
|R7 | The FE must manage concurrent get/set by multiple threads (Thread safety)|
|R8 | The FE must manage concurrent arm/disarm by multiple threads  (Thread safety)|
|R9 | The FE must allow exploration of the actual configuration|
|R10 | The FE should be able to control the configuration state via the back-ends (existing, corrupted, non existing)|

### Interface methods

The unsafe (type-unsafe) methods does not use the proper data structure for the set and get but instead just pass a parameter value for a specific registry entry enumerator index (e.g. ignition time)

- **setConfiguration[Unsafe]**:  A setter method is in need to ensure R1. This method should allow 
    insert a value for a specific configuration entry while guarantee the different data types for the values (R3).
    
- **getConfiguration[Unsafe]**:  A getter method is in need to ensure the visibility of the configuration. 
    Such method should get a value for a specified configuration entry and changes the value to passed by reference value parameter.

- **getOrSetDefaultConfiguration[Unsafe]**: A particular get method which returns the get value and if it is not existing in the configuration set it (if possible!) and returns the default value.

 - **arm**:  An "arm" method which guarantees no memory allocations and no changes to the configuration are in place
    until a "disarm" method. It is an important functionality to ensure a "safe" memory configuration during flight.

 - **disarm**: A "disarm" method, to disable the stable and "safe" configuration mode of the registry to allow again 
    allocations and settings on the configuration.

 - **isConfigured**: A method to explore the actual status of the configuration, if there is an existing one in memory or not.

 - **isConfigurationEmpty**: A method to know if there is an existing configuration or if it is empty

 - **isConfigurationCorrupted**: A method to know if there is an existing configuration in the registry but corrupted memory.

- **configuredEntries**: A method which returns the already existing entries of the configuration as a set.

### Data structures
The data structures are managed in 2 main header files.
#### TypeStructures.h
Type structures have:

- **RootTypeStructure**: A root type, with just 2 template attributes: value and index

- **(Float|UInt32|...)Type**: A sub-type that does specify the value attribute type. It inherits from RootTypeStructure

#### RegistryStructures.h
Registry structures contains:

- **ConfigurationEnum**: The enumerator with the possible configurations entries for the registry

- **TypeUnion**: The union type for saving the values of the different configuration entries

- **UnionWrap(Float|UInt32|...)Type**: The structures inheriting from (Float|UInt32|...)Type that includes methods to consistently manage the unionType variables to set them or get their correct type field.

- **(Ignition|DeploymentAltitude|...)**: All the specific data structures for the configuration entries. They does specify the correct enum index and inherit correctly from the union wrapper.

