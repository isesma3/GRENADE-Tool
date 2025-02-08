# SANTA
23-24 VGC Mission to Titan
How-to:
To run the code, all you have to do is go to the Inp folder and fill out the Input.csv, Mission_Profile.csv, and Configuration_Input.csv files with mission requirements and configurations. Once this is done, all you have to do is run the Main_v5.py file. The code takes a while to run so go work out or something while it's running :) <3

If you're getting "Table values replaced with NaN", do this.
    If you're working with VSCode and you want to get rid of "Missing values blabla"
1) Go to the AnS_BWB.py file
2) Right click on avlwrapper (in the import) and "go to definition"
3) At the top, where the path is written, click on "__init__.py" and look for output.py (like in the screenshot)
4) In output.py, find the line "logger.warning("Table values missing. Replaced with NaN")"
5) Comment that line (with #)
6) Save file