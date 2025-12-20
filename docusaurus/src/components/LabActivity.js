import React from 'react';
import clsx from 'clsx';
import styles from './LabActivity.module.css';

const LabActivity = ({ title, time, difficulty, children }) => {
  return (
    <div className={clsx('lab-activity-container', styles.container)}>
      <div className={styles.header}>
        <div className={styles.titleSection}>
          <span className={styles.icon}>🧪</span>
          <h3 className={styles.title}>{title || 'Lab Activity'}</h3>
        </div>
        <div className={styles.meta}>
          {time && <span className={styles.time}>⏱️ {time}</span>}
          {difficulty && <span className={styles.difficulty}>🎯 {difficulty}</span>}
        </div>
      </div>
      <div className={styles.content}>
        {children}
      </div>
    </div>
  );
};

export default LabActivity;