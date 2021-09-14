#!/bin/bash

# Input checks
if [ "$#" -ne 1 ]; then
   echo "Usage: scxml2plant <FOLDER_TO_SEARCH>"
   exit 1
fi

# Find this folder when calling the script from another folder
PATH_TO_XALAN=$(dirname $0)

# Select all scxml files
for file in $(find $1 -name "*.scxml")
do
    # generate plantuml
    INPUT_XML=$file
    OUTPUT_NAME=$(dirname $file)/$(basename -s ".scxml" $file).plantuml
    echo "Input file: " $INPUT_XML " Output basename: " $OUTPUT_NAME

    java -cp $PATH_TO_XALAN/xalan/xalan.jar \
    -Dorg.apache.xerces.xni.parser.XMLParserConfiguration=org.apache.xerces.parsers.XIncludeParserConfiguration org.apache.xalan.xslt.Process \
    -IN $INPUT_XML -XSL $PATH_TO_XALAN/xslt/scxml2plantuml.xslt -OUT $OUTPUT_NAME

    # generate README
    README_FILE=$(dirname $file)/README.md

    touch $README_FILE
    echo "\`\`\`plantuml" > $README_FILE
    cat $OUTPUT_NAME >> $README_FILE
    echo "\`\`\`" >> $README_FILE
    rm $OUTPUT_NAME
done
