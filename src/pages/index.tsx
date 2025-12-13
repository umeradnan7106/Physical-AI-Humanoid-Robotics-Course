import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

const modules = [
  {
    title: '🌟 Introduction',
    emoji: '🌟',
    gradient: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    chapters: [
      { title: 'What is Physical AI?', link: '/docs/introduction/what-is-physical-ai' },
      { title: 'Why Physical AI Matters', link: '/docs/introduction/why-physical-ai-matters' },
      { title: 'Digital to Embodied', link: '/docs/introduction/digital-to-embodied' },
      { title: 'Humanoid Robotics Landscape', link: '/docs/introduction/humanoid-robotics-landscape' },
      { title: 'Sensor Systems Overview', link: '/docs/introduction/sensor-systems-overview' },
    ]
  },
  {
    title: '🚀 Getting Started',
    emoji: '🚀',
    gradient: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
    chapters: [
      { title: 'Hardware Requirements', link: '/docs/getting-started/hardware-requirements' },
      { title: 'Environment Setup', link: '/docs/getting-started/environment-setup' },
      { title: 'Cloud Alternatives', link: '/docs/getting-started/cloud-alternatives' },
    ]
  },
  {
    title: '🤖 Module 1: ROS 2 Fundamentals',
    emoji: '🤖',
    gradient: 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)',
    chapters: [
      { title: 'ROS 2 Architecture', link: '/docs/module-01-ros2/ros2-architecture' },
      { title: 'Nodes, Topics & Services', link: '/docs/module-01-ros2/nodes-topics-services' },
      { title: 'URDF for Humanoids', link: '/docs/module-01-ros2/urdf-humanoids' },
      { title: 'Launch Files', link: '/docs/module-01-ros2/launch-files' },
      { title: 'Building Packages', link: '/docs/module-01-ros2/building-packages' },
      { title: 'Project: ROS 2 Package', link: '/docs/module-01-ros2/project-ros2-package' },
    ]
  },
  {
    title: '🎮 Module 2: Simulation',
    emoji: '🎮',
    gradient: 'linear-gradient(135deg, #43e97b 0%, #38f9d7 100%)',
    chapters: [
      { title: 'Gazebo Fundamentals', link: '/docs/module-02-simulation/gazebo-fundamentals' },
      { title: 'Spawning Robots', link: '/docs/module-02-simulation/spawning-robots' },
      { title: 'Sensor Integration', link: '/docs/module-02-simulation/sensor-integration' },
      { title: 'Unity Robotics', link: '/docs/module-02-simulation/unity-robotics' },
      { title: 'Project: Simulation', link: '/docs/module-02-simulation/project-simulation' },
    ]
  },
  {
    title: '🎯 Module 3: Isaac Sim',
    emoji: '🎯',
    gradient: 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)',
    chapters: [
      { title: 'Isaac Sim Setup', link: '/docs/module-03-isaac/isaac-sim-setup' },
      { title: 'Visual SLAM', link: '/docs/module-03-isaac/vslam-visual-slam' },
      { title: 'Nav2 Navigation', link: '/docs/module-03-isaac/nav2-navigation' },
      { title: 'Isaac ROS Perception', link: '/docs/module-03-isaac/isaac-ros-perception' },
      { title: 'Project: Navigation', link: '/docs/module-03-isaac/project-isaac-navigation' },
    ]
  },
  {
    title: '🧠 Module 4: VLA Integration',
    emoji: '🧠',
    gradient: 'linear-gradient(135deg, #30cfd0 0%, #330867 100%)',
    chapters: [
      { title: 'Whisper Integration', link: '/docs/module-04-vla/whisper-integration' },
      { title: 'LLM Planning', link: '/docs/module-04-vla/llm-planning' },
      { title: 'Ollama Local LLMs', link: '/docs/module-04-vla/ollama-local-llms' },
      { title: 'Action Execution', link: '/docs/module-04-vla/action-execution' },
      { title: 'Project: VLA Pipeline', link: '/docs/module-04-vla/project-vla-pipeline' },
    ]
  },
  {
    title: '🏆 Capstone Project',
    emoji: '🏆',
    gradient: 'linear-gradient(135deg, #a8edea 0%, #fed6e3 100%)',
    chapters: [
      { title: 'System Architecture', link: '/docs/capstone/system-architecture' },
      { title: 'Voice to Manipulation', link: '/docs/capstone/voice-to-manipulation' },
      { title: 'Integration Guide', link: '/docs/capstone/integration-guide' },
      { title: 'Demo Presentation', link: '/docs/capstone/demo-presentation' },
    ]
  }
];

function ModuleCard({module}) {
  return (
    <div className={styles.moduleCard}>
      <div className={styles.moduleHeader} style={{background: module.gradient}}>
        <div className={styles.moduleEmoji}>{module.emoji}</div>
        <h2 className={styles.moduleTitle}>{module.title}</h2>
      </div>
      <div className={styles.chapterList}>
        {module.chapters.map((chapter, idx) => (
          <Link key={idx} to={chapter.link} className={styles.chapterLink}>
            <div className={styles.chapterCard}>
              <span className={styles.chapterNumber}>{idx + 1}</span>
              <span className={styles.chapterTitle}>{chapter.title}</span>
              <span className={styles.chapterArrow}>→</span>
            </div>
          </Link>
        ))}
      </div>
    </div>
  );
}

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <Heading as="h1" className={styles.heroTitle}>
            {siteConfig.title}
          </Heading>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <div className={styles.heroDescription}>
            <p>Master the future of robotics with hands-on projects covering ROS 2, simulation, NVIDIA Isaac Sim, and Vision-Language-Action models.</p>
          </div>
          <div className={styles.buttons}>
            <Link
              className="button button--primary button--lg"
              to="/docs/intro">
              Start Learning 🚀
            </Link>
            <Link
              className="button button--secondary button--lg"
              to="https://github.com/umeradnan7106/Physical-AI-Humanoid-Robotics-Course">
              View on GitHub ⭐
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A comprehensive hands-on guide to Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main className={styles.mainContent}>
        <div className="container">
          <div className={styles.modulesSection}>
            <h2 className={styles.sectionTitle}>📚 Course Modules</h2>
            <p className={styles.sectionSubtitle}>Click on any chapter to start learning</p>
            <div className={styles.modulesGrid}>
              {modules.map((module, idx) => (
                <ModuleCard key={idx} module={module} />
              ))}
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
