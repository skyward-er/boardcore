# Events Generator Script

This script generates the Events.h and Topics.h heander file from a GoogleSheets document on Google Drive.

To execute the script:

```
pip install --upgrade google-api-python-client
pip install oauth2client
python scripts/homeone/event_header_generator/event_generator.py
```

The first time a browser window should open asking for your credentials (use the Skyward ones).
If it doesn't and only gives you an error, try executing it by clicking on the icon.