# Data

This directory stores simulation input today. The general recommended folder structure is as follows

```
|- data
    |- location
        |- bathymetry
        |- ssp
        |- current
```

A good example of the directory structure used can be found in `sim_config/`

# Important Considerations
Data should have NaN values filled with appropriate information to prevent C++ from having to handle all that complexity.
For current data, all NaN values were filled with 0, while for SSP ZoH was done to fill the values below bathymetry.