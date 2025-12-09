import React from 'react';
import CodeBlock from '@theme/CodeBlock';

export interface CustomCodeBlockProps {
  children: string;
  language?: string;
  title?: string;
  showLineNumbers?: boolean;
}

/**
 * Enhanced code block component for Physical AI book
 * Supports Python, Bash, YAML, XML (for URDF/launch files)
 */
export default function CustomCodeBlock({
  children,
  language = 'python',
  title,
  showLineNumbers = true,
}: CustomCodeBlockProps): JSX.Element {
  return (
    <CodeBlock
      language={language}
      title={title}
      showLineNumbers={showLineNumbers}
    >
      {children}
    </CodeBlock>
  );
}
