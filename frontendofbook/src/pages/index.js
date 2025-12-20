import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Accordion from '@site/src/components/Accordion';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Read the Book
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/ros2-humanoid/intro-to-ros2">
            Start with Module-01 ROS 2
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeaturesSection() {
  const features = [
    {
      icon: '🤖',
      title: 'AI-Powered Learning',
      description: 'Our book combines traditional documentation with AI-powered assistance to provide contextual answers based on the content, ensuring you get relevant information quickly.'
    },
    {
      icon: '📚',
      title: 'Modular Curriculum',
      description: 'Learn through structured modules, starting with the fundamentals of ROS 2 and progressing to advanced humanoid robotics concepts.'
    },
    {
      icon: '🔍',
      title: 'Grounded Responses',
      description: 'All AI responses are grounded in actual documentation to ensure accuracy and prevent hallucinations, providing reliable information.'
    }
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h2>{feature.title}</h2>
              <p>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  const modules = [
    {
      icon: '🔧',
      title: 'Module 1',
      subtitle: 'ROS 2 Fundamentals',
      description: 'Introduction to ROS 2, DDS concepts, and the middleware nervous system for humanoid robots.',
      path: '/docs/ros2-humanoid/intro-to-ros2'
    },
    {
      icon: '👁️',
      title: 'Module 2',
      subtitle: 'Perception Systems',
      description: 'Learn about sensors, computer vision, and perception algorithms for humanoid robots.',
      path: '/docs/digital-twin/sensor-simulation-validation'
    },
    {
      icon: '🚶',
      title: 'Module 3',
      subtitle: 'Motion Planning',
      description: 'Understand locomotion, balance, and movement planning for humanoid robots.',
      path: '/docs/isaac-ai-brain/nav2-path-planning-humanoid'
    },
    {
      icon: '🤝',
      title: 'Module 4',
      subtitle: 'Human-Robot Interaction',
      description: 'Explore interfaces, communication, and interaction patterns for humanoid robots.',
      path: '/docs/vla-llm-integration/cognitive-planning-llms-ros2'
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2>Learning Modules</h2>
        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <div key={index} className={styles.moduleCard}>
              <div className={styles.moduleIcon}>{module.icon}</div>
              <h3>{module.title}</h3>
              <h4>{module.subtitle}</h4>
              <p>{module.description}</p>
              <Link to={module.path} className="button button--primary">
                Start Learning
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HowItWorksSection() {
  const steps = [
    {
      title: "Select a Module",
      description: "Choose from our structured learning modules based on your current knowledge level."
    },
    {
      title: "Learn Concepts",
      description: "Study the documentation with interactive examples and practical exercises."
    },
    {
      title: "Ask Questions",
      description: "Use our AI assistant to get contextual answers grounded in the documentation."
    }
  ];

  return (
    <section className={styles.howItWorksSection}>
      <div className="container">
        <h2>How It Works</h2>
        <div className={styles.workflowContainer}>
          <div className={styles.workflowSteps}>
            {steps.map((step, index) => (
              <div
                key={index}
                className={styles.workflowStep}
                data-step={index + 1}
              >
                <div className={styles.workflowStepContent}>
                  <h3>{step.title}</h3>
                  <p>{step.description}</p>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

function FAQSection() {
  return (
    <section className={styles.faqSection}>
      <div className="container">
        <h2>Frequently Asked Questions</h2>
        <div className={styles.faqLayout}>
          <div className={styles.faqImage}>
            Support & Help
          </div>
          <div className={styles.faqContainer}>
            <Accordion />
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="An AI-powered book with embedded RAG chatbot using Spec-Driven Development">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <ModulesSection />
        <HowItWorksSection />
        <FAQSection />
      </main>
    </Layout>
  );
}