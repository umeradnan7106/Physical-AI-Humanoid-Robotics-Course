import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

export interface TabItemConfig {
  value: string;
  label: string;
  content: React.ReactNode;
}

export interface CustomTabsProps {
  items: TabItemConfig[];
  defaultValue?: string;
  groupId?: string;
}

/**
 * Custom tabs component for organizing multi-option content
 * Used for: Cloud vs Local setup, Different LLM providers, Gazebo vs Unity
 */
export default function CustomTabs({
  items,
  defaultValue,
  groupId,
}: CustomTabsProps): JSX.Element {
  return (
    <Tabs defaultValue={defaultValue || items[0]?.value} groupId={groupId}>
      {items.map((item) => (
        <TabItem key={item.value} value={item.value} label={item.label}>
          {item.content}
        </TabItem>
      ))}
    </Tabs>
  );
}
