import React from 'react';
import clsx from 'clsx';
import styles from './Assessment.module.css';

const Assessment = ({ type = "quiz", title, objectives, rubric, children }) => {
  return (
    <div className={clsx('assessment-container', styles.container)}>
      <div className={styles.header}>
        <div className={styles.titleSection}>
          <span className={styles.icon}>📝</span>
          <h3 className={styles.title}>{type.charAt(0).toUpperCase() + type.slice(1)}: {title}</h3>
        </div>
      </div>

      {objectives && objectives.length > 0 && (
        <div className={styles.objectivesSection}>
          <h4 className={styles.sectionTitle}>Learning Objectives</h4>
          <ul className={styles.objectivesList}>
            {objectives.map((obj, index) => (
              <li key={index} className={styles.objectiveItem}>{obj}</li>
            ))}
          </ul>
        </div>
      )}

      {rubric && (
        <div className={styles.rubricSection}>
          <h4 className={styles.sectionTitle}>Assessment Rubric</h4>
          <div className={styles.rubricGrid}>
            {rubric.map((criterion, index) => (
              <div key={index} className={styles.rubricItem}>
                <div className={styles.criterion}>{criterion.criterion}</div>
                <div className={styles.scores}>
                  {criterion.scores.map((score, idx) => (
                    <div key={idx} className={styles.score}>
                      <span className={styles.scoreLabel}>{score.label}:</span>
                      <span className={styles.scoreValue}>{score.value}</span>
                    </div>
                  ))}
                </div>
              </div>
            ))}
          </div>
        </div>
      )}

      <div className={styles.content}>
        {children}
      </div>
    </div>
  );
};

export default Assessment;