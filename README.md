# wisevision_data_black_box
This package is a part of tools useful for the WiseVision Dashboard, proividing a database manager.

Manager handle getting and deleting data in database and managing Influx database through zenoh server. 

Used in the WiseVision Dashboard for charts and reports.

## Table of contents
For more information, please refer to the following sections:

[Build and run(Start here)](docs/BUILD.md)

[Minimal example](docs/MINIMAL_EXAMPLE.md)

If you want to contribute to this project, please read the following sections:

[Code of conduct](docs/CODE_OF_CONDUCT.md)

## API
### Services
- /get_messages
- /create_database
- /add_storage_to_database
- /add_data_to_database
- /delete_data_from_database