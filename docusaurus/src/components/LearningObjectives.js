import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

const LearningObjectives = ({ objectives, title = "Learning Objectives" }) => {
  return (
    <div className={clsx('learning-objectives-container', styles.container)}>
      <div className={styles.header}>
        <span className={styles.icon}>🎯</span>
        <h3 className={styles.title}>{title}</h3>
      </div>
      <ul className={styles.objectivesList}>
        {objectives.map((objective, index) => (
          <li key={index} className={styles.objectiveItem}>
            {objective}
          </li>
        ))}
      </ul>
    </div>
  );
};

export default LearningObjectives;