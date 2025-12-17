import React from 'react';
import clsx from 'clsx';
import styles from './Glossary.module.css';

const Glossary = ({ terms, title = "Glossary" }) => {
  return (
    <div className={clsx('glossary-container', styles.container)}>
      <div className={styles.header}>
        <span className={styles.icon}>📖</span>
        <h3 className={styles.title}>{title}</h3>
      </div>
      <dl className={styles.termsList}>
        {terms.map((term, index) => (
          <div key={index} className={styles.termItem}>
            <dt className={styles.termName}>{term.name}</dt>
            <dd className={styles.termDefinition}>{term.definition}</dd>
          </div>
        ))}
      </dl>
    </div>
  );
};

export default Glossary;