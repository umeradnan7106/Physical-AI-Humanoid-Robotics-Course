#!/usr/bin/env node
/**
 * Frontmatter Schema Validator for Physical AI Book
 * Validates all .md/.mdx files have required frontmatter fields
 */

const fs = require('fs');
const path = require('path');
const matter = require('gray-matter');

// Required frontmatter fields per content type
const REQUIRED_FIELDS = {
  chapter: ['id', 'title', 'sidebar_label', 'sidebar_position', 'description'],
  tutorial: ['id', 'title', 'sidebar_position', 'description'],
  project: ['id', 'title', 'sidebar_position', 'description'],
};

function validateFrontmatter(filePath) {
  try {
    const content = fs.readFileSync(filePath, 'utf8');
    const { data: frontmatter } = matter(content);

    // Determine content type from file
    const contentType = filePath.includes('project') ? 'project' :
                       filePath.includes('tutorial') ? 'tutorial' : 'chapter';

    const required = REQUIRED_FIELDS[contentType];
    const missing = required.filter(field => !frontmatter[field]);

    if (missing.length > 0) {
      console.error(`❌ ${filePath}: Missing fields: ${missing.join(', ')}`);
      return false;
    }

    console.log(`✅ ${filePath}: Valid frontmatter`);
    return true;
  } catch (error) {
    console.error(`❌ ${filePath}: Error reading file - ${error.message}`);
    return false;
  }
}

function findMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir, { withFileTypes: true });

  for (const item of items) {
    const fullPath = path.join(dir, item.name);
    if (item.isDirectory()) {
      files.push(...findMarkdownFiles(fullPath));
    } else if (item.name.endsWith('.md') || item.name.endsWith('.mdx')) {
      files.push(fullPath);
    }
  }

  return files;
}

// Main execution
const docsDir = path.join(__dirname, '..', 'docs');
const files = findMarkdownFiles(docsDir);
const results = files.map(validateFrontmatter);
const passed = results.filter(r => r).length;

console.log(`\n📊 Summary: ${passed}/${files.length} files passed validation`);
process.exit(passed === files.length ? 0 : 1);
