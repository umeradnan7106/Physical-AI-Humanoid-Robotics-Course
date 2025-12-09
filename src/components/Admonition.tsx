import React from 'react';
import Admonition from '@theme/Admonition';

export type AdmonitionType = 'note' | 'tip' | 'info' | 'caution' | 'danger';

export interface CustomAdmonitionProps {
  type: AdmonitionType;
  title?: string;
  children: React.ReactNode;
}

/**
 * Custom admonition component for callouts, warnings, tips
 * Used for: Common Pitfalls, Hardware Requirements, Troubleshooting
 */
export default function CustomAdmonition({
  type,
  title,
  children,
}: CustomAdmonitionProps): JSX.Element {
  return (
    <Admonition type={type} title={title}>
      {children}
    </Admonition>
  );
}
