INPUT_FILE=$1
OUTPUT_FILE=$2.inl

echo "#pragma once" > "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"
echo "static BYTE _$2[] = {" >> "$OUTPUT_FILE"

# dump the entire file as hex codes using a ":" as a placeholder for a newline
# then pipe that through sed to remove the last comma and any empty trailing records
# and finally, convert the placeholders back into real newlines
# have to use a placeholder so the next command doesn't process every individual file
hexdump -v -e '16/1 "0x%02X," ":"' "$INPUT_FILE" | sed 's/,[0x  ,]*:$//' | sed 's/:/\n/g' >> "$OUTPUT_FILE"
# ^------------------- dump -------------------^   ^- remove trailing -^   ^- newlines -^

echo "};" >> "$OUTPUT_FILE"
