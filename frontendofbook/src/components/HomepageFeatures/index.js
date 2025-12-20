import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'AI-Powered Documentation',
    description: (
      <>
        Our book combines traditional documentation with AI-powered assistance to provide contextual answers based on the content.
      </>
    ),
  },
  {
    title: 'Spec-Driven Approach',
    description: (
      <>
        Built using Spec-Driven Development methodology with the Spec-Kit Plus framework for structured development.
      </>
    ),
  },
  {
    title: 'Retrieval-Augmented Generation',
    description: (
      <>
        Responses are grounded in your actual documentation to ensure accuracy and prevent hallucinations.
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}