import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FAQ_DATA = [
  {
    question: "Who is this for?",
    answer: "AI students and developers entering humanoid robotics who want to learn ROS 2 concepts in a structured, interactive way."
  },
  {
    question: "Do I need prior robotics experience?",
    answer: "No, the modules start with fundamentals and gradually build up to advanced concepts, making it accessible for beginners."
  },
  {
    question: "How is this different from other ROS 2 tutorials?",
    answer: "Our approach combines structured learning modules with AI-powered assistance that provides contextual answers grounded in the documentation."
  },
  {
    question: "Can I contribute to the content?",
    answer: "Yes! This project follows a spec-driven development approach, and contributions are welcome through the GitHub repository."
  }
];

function AccordionItem({ question, answer, isOpen, onToggle }) {
  return (
    <div className={clsx(styles.faqItem, { [styles.active]: isOpen })}>
      <button
        className={styles.faqQuestion}
        onClick={onToggle}
        aria-expanded={isOpen}
        aria-controls={question.toLowerCase().replace(/\s+/g, '-')}
      >
        <span>{question}</span>
        <span className={styles.faqIcon}>▼</span>
      </button>
      <div
        id={question.toLowerCase().replace(/\s+/g, '-')}
        className={clsx(styles.faqAnswer, { [styles.expanded]: isOpen })}
      >
        <p>{answer}</p>
      </div>
    </div>
  );
}

export default function Accordion() {
  const [openIndex, setOpenIndex] = useState(null);

  const handleToggle = (index) => {
    setOpenIndex(openIndex === index ? null : index);
  };

  return (
    <div className={styles.accordion}>
      {FAQ_DATA.map((faq, index) => (
        <AccordionItem
          key={index}
          question={faq.question}
          answer={faq.answer}
          isOpen={openIndex === index}
          onToggle={() => handleToggle(index)}
        />
      ))}
    </div>
  );
}