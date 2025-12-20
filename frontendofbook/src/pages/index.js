import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

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
            className="button button--primary button--lg margin-left--sm"
            to="/docs/ros2-humanoid/intro-to-ros2">
            Start with Module-01 ROS 2
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className="container padding-vert--lg">
        <div className="row">
          <div className="col col--4">
            <h2>🤖 AI-Powered Learning</h2>
            <p>
              Our book combines traditional documentation with AI-powered assistance to provide contextual answers based on the content, ensuring you get relevant information quickly.
            </p>
          </div>
          <div className="col col--4">
            <h2>📚 Modular Curriculum</h2>
            <p>
              Learn through structured modules, starting with the fundamentals of ROS 2 and progressing to advanced humanoid robotics concepts.
            </p>
          </div>
          <div className="col col--4">
            <h2>🔍 Grounded Responses</h2>
            <p>
              All AI responses are grounded in actual documentation to ensure accuracy and prevent hallucinations, providing reliable information.
            </p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ModulesSection() {
  return (
    <section className={clsx(styles.modulesSection, 'bg--secondary')}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-bottom--lg">Learning Modules</h2>
        <div className="row">
          <div className="col col--3">
            <div className="text--center padding-horiz--md">
              <h3>Module 1</h3>
              <h4>ROS 2 Fundamentals</h4>
              <p>Introduction to ROS 2, DDS concepts, and the middleware nervous system for humanoid robots.</p>
              <Link to="/docs/ros2-humanoid/intro-to-ros2" className="button button--primary button--block">
                Start Learning
              </Link>
            </div>
          </div>
          <div className="col col--3">
            <div className="text--center padding-horiz--md">
              <h3>Module 2</h3>
              <h4>Perception Systems</h4>
              <p>Learn about sensors, computer vision, and perception algorithms for humanoid robots.</p>
              <Link to="/docs/intro" className="button button--secondary button--block button--disabled">
                Coming Soon
              </Link>
            </div>
          </div>
          <div className="col col--3">
            <div className="text--center padding-horiz--md">
              <h3>Module 3</h3>
              <h4>Motion Planning</h4>
              <p>Understand locomotion, balance, and movement planning for humanoid robots.</p>
              <Link to="/docs/intro" className="button button--secondary button--block button--disabled">
                Coming Soon
              </Link>
            </div>
          </div>
          <div className="col col--3">
            <div className="text--center padding-horiz--md">
              <h3>Module 4</h3>
              <h4>Human-Robot Interaction</h4>
              <p>Explore interfaces, communication, and interaction patterns for humanoid robots.</p>
              <Link to="/docs/intro" className="button button--secondary button--block button--disabled">
                Coming Soon
              </Link>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function HowItWorksSection() {
  return (
    <section className={styles.howItWorksSection}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-bottom--lg">How It Works</h2>
        <div className="row">
          <div className="col col--4">
            <div className="text--center">
              <h3>1. Select a Module</h3>
              <p>Choose from our structured learning modules based on your current knowledge level.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center">
              <h3>2. Learn Concepts</h3>
              <p>Study the documentation with interactive examples and practical exercises.</p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center">
              <h3>3. Ask Questions</h3>
              <p>Use our AI assistant to get contextual answers grounded in the documentation.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

function FAQSection() {
  return (
    <section className={clsx(styles.faqSection, 'bg--secondary')}>
      <div className="container padding-vert--lg">
        <h2 className="text--center padding-bottom--lg">Frequently Asked Questions</h2>
        <div className="row">
          <div className="col col--6">
            <h3>Who is this for?</h3>
            <p>AI students and developers entering humanoid robotics who want to learn ROS 2 concepts in a structured, interactive way.</p>
          </div>
          <div className="col col--6">
            <h3>Do I need prior robotics experience?</h3>
            <p>No, the modules start with fundamentals and gradually build up to advanced concepts, making it accessible for beginners.</p>
          </div>
        </div>
        <div className="row padding-top--lg">
          <div className="col col--6">
            <h3>How is this different from other ROS 2 tutorials?</h3>
            <p>Our approach combines structured learning modules with AI-powered assistance that provides contextual answers grounded in the documentation.</p>
          </div>
          <div className="col col--6">
            <h3>Can I contribute to the content?</h3>
            <p>Yes! This project follows a spec-driven development approach, and contributions are welcome through the GitHub repository.</p>
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