#!/bin/bash

# Project snapshot script for C projects

output_file="project_snapshot.txt"
snapshot_dir="${1:-.}"

# Extensions to exclude (C-specific and snapshot-related)
exclude_extensions=("o" "so" "a" "log" "env" "txt" "js" "$output_file" "sh" "blob" "zip")

# Folders to exclude 
exclude_folders=("node_modules" ".git" "dist" "test" "build" "devel" "cmake")

# Enable error checking
set -e  # Exit immediately if a command exits with a non-zero status.
set -u  # Treat unset variables as an error.

# Redirect stderr to stdout to capture error messages
2>&1 

# Add more verbose output for debugging
set -x  # Print commands and their arguments as they are executed.


# Function to determine file type (add C-related types)
get_file_type() {
    local file="$1"
    local extension="${file##*.}"
    case "$extension" in
        c)      echo "C Source" ;;
        h)      echo "C Header" ;;
        js)     echo "JavaScript" ;;
        vue)    echo "Vue Component" ;;
        json)   echo "JSON" ;;
        md)     echo "Markdown" ;;
        sh)     echo "Shell Script" ;;
        conf)   echo "Configuration File" ;;
        html)   echo "HTML" ;;
        css)    echo "CSS" ;;
        yml|yaml) echo "YAML" ;;
        Dockerfile) echo "Dockerfile" ;;
        *)      
                if file "$file" | grep -q "ELF" >/dev/null 2>&1; then
                        echo "Object/Library File" 
                elif file -b "$file" >/dev/null 2>&1; then
                         echo "Binary File"
                else 
                       echo "Unknown" 
                fi;;
    esac
}

# Function to generate file tree (improved exclusion logic)
generate_tree() {
    local dir="$1"
    local prefix="$2"
    local is_last="$3"

    local items=($(find "$dir" -maxdepth 1 -mindepth 1 -print0 | sort -z))
    local total=${#items[@]}
    local count=0

    for item in "${items[@]}"; do
        ((count++))
        local base_name=$(basename "$item")

        skip_file=0
        for ext in "${exclude_extensions[@]}"; do
            if [[ "$item" == *".$ext" || "$base_name" == "$ext" ]]; then 
                 skip_file=1
                 break
            fi
        done

        for folder in "${exclude_folders[@]}"; do
            if [[ "$base_name" == "$folder" ]]; then
                skip_file=1
                break
            fi
        done

       if (( skip_file == 1 )); then
            continue
       fi

        local is_last_item=$([[ $count -eq $total ]] && echo "1" || echo "0")
        local item_prefix=$([[ "$is_last" == "1" ]] && echo "    " || echo "│   ")
        local symbol=$([[ "$is_last_item" == "1" ]] && echo "└── " || echo "├── ")

        if [ -d "$item" ]; then
            echo -n "${prefix}${symbol}${base_name}/\n" >> "$output_file" 
            generate_tree "$item" "${prefix}${item_prefix}" "$is_last_item" 
        elif [ -f "$item" ]; then
           echo -n "${prefix}${symbol}${base_name}\n" >> "$output_file"
        fi
    done
}

# Function to print file contents with metadata (improved exclusions)
print_file_contents() {
    local root_dir="$1"
    local current_dir="$2"

    for item in "$current_dir"/*; do 
        local base_name=$(basename "$item")

        skip_file=0
        for ext in "${exclude_extensions[@]}"; do
            if [[ "$item" == *".$ext" || "$base_name" == "$ext" ]]; then
                 skip_file=1
                break
           fi
        done

        for folder in "${exclude_folders[@]}"; do
            if [[ "$base_name" == "$folder" ]]; then
                skip_file=1
                break
            fi
        done

       if (( skip_file == 1 )); then
            continue 
       fi

        if [[ -d "$item" ]]; then
            print_file_contents "$root_dir" "$item"
        elif [ -f "$item" ]; then
            local relative_path="${item#$root_dir/}"
            local file_type=$(get_file_type "$item")
            local file_size=$(wc -c < "$item")
            local last_modified=$(date -r "$item" "+%Y-%m-%d %H:%M:%S")
            
            echo "<<<FILE_START>>>" >> "$output_file"
            echo "Path: $relative_path" >> "$output_file"
            echo "Type: $file_type" >> "$output_file"
            echo "Size: $file_size bytes" >> "$output_file"
            echo "Last Modified: $last_modified" >> "$output_file"
            echo "<<<CONTENT>>>" >> "$output_file"
            cat "$item" >> "$output_file"
            echo "<<<FILE_END>>>" >> "$output_file"
            echo "" >> "$output_file"
        fi
    done
}

# Main execution (using $output_file consistently)
root_dir="$(pwd)"
project_name=$(basename "$root_dir")

file_count=$(find . -type f -print0 | while IFS= read -r -d $'\0' file; do
    skip=0
    for ext in "${exclude_extensions[@]}"; do
        if [[ "$file" == *".$ext" ]]; then
            skip=1
            break  
        fi
    done
    for folder in "${exclude_folders[@]}"; do
        if [[ "$file" == *"/$folder/"* ]]; then
            skip=1
            break
        fi
    done
    if (( skip == 0 )); then
        echo "$file"
    fi
done | wc -l)

dir_count=$(find . -type d -print0 | while IFS= read -r -d $'\0' dir; do
    skip=0
    for ext in "${exclude_extensions[@]}"; do 
        base=$(basename "$dir")
        if [[ "$base" == "$ext" ]]; then
            skip=1
            break
        fi
    done
    for folder in "${exclude_folders[@]}"; do
        if [[ "$dir" == *"/$folder"* ]]; then
            skip=1
            break
        fi
    done
    if (( skip == 0 )); then
        echo "$dir"
    fi
done | wc -l)


echo "<<<PROJECT_START>>>" > "$output_file"
echo "Project: $project_name" >> "$output_file"
echo "Root Directory: $root_dir" >> "$output_file"
echo "File Count: $file_count" >> "$output_file"
echo "Directory Count: $dir_count" >> "$output_file"
echo "" >> "$output_file"

echo "<<<TABLE_OF_CONTENTS>>>" >> "$output_file"
find . -type f -print0 | while IFS= read -r -d $'\0' file; do
    skip=0
    for ext in "${exclude_extensions[@]}"; do
        if [[ "$file" == *".$ext" ]]; then
            skip=1
            break
        fi
    done
    for folder in "${exclude_folders[@]}"; do
        if [[ "$file" == *"/$folder/"* ]]; then
            skip=1
            break
        fi
    done
    if (( skip == 0 )); then
        echo "$file"
    fi
done | sort | sed 's|^./||' | awk '{print NR ". " $0}' >> "$output_file"
echo "<<<END_TABLE_OF_CONTENTS>>>" >> "$output_file"
echo "" >> "$output_file"

echo "<<<FILE_TREE>>>" >> "$output_file"
generate_tree "." "" "0" >> "$output_file"
echo "<<<END_FILE_TREE>>>" >> "$output_file"
echo "" >> "$output_file"

print_file_contents "$root_dir" "$root_dir"

echo "<<<PROJECT_END>>>" >> "$output_file"

echo "Project structure and file contents have been written to $output_file"