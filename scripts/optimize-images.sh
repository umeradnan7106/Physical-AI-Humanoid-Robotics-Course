#!/bin/bash
# Image Optimization Pipeline for Physical AI Book
# Target: <500KB per image

echo "📸 Optimizing images in static/img/"

# Check if imagemagick is installed
if ! command -v convert &> /dev/null; then
    echo "⚠️  ImageMagick not found. Install with: sudo apt-get install imagemagick"
    exit 1
fi

# Find all PNG/JPG files larger than 500KB
find static/img -type f \( -name "*.png" -o -name "*.jpg" -o -name "*.jpeg" \) -size +500k | while read file; do
    echo "Optimizing: $file"

    # Create backup
    cp "$file" "$file.bak"

    # Optimize PNG
    if [[ $file == *.png ]]; then
        convert "$file" -strip -resize '1920x1080>' -quality 85 "$file"
    fi

    # Optimize JPEG
    if [[ $file == *.jpg ]] || [[ $file == *.jpeg ]]; then
        convert "$file" -strip -resize '1920x1080>' -quality 80 "$file"
    fi

    # Check new size
    new_size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null)
    if [ $new_size -gt 512000 ]; then
        echo "  ⚠️  Still >500KB: $new_size bytes"
    else
        echo "  ✅ Optimized to: $new_size bytes"
        rm "$file.bak"
    fi
done

echo "✨ Image optimization complete"
