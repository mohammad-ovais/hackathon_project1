import React from 'react';
import clsx from 'clsx';
import styles from './Exercise.module.css';

const ExerciseType = {
  EXERCISE: 'exercise',
  LAB: 'lab',
  ASSESSMENT: 'assessment',
  CHECKLIST: 'checklist'
};

const Exercise = ({ type, title, children, difficulty = 'medium' }) => {
  const exerciseType = type || ExerciseType.EXERCISE;

  const getExerciseConfig = () => {
    switch(exerciseType) {
      case ExerciseType.LAB:
        return {
          className: styles.lab,
          icon: '🧪',
          title: `Lab Activity: ${title || 'Lab Exercise'}`
        };
      case ExerciseType.ASSESSMENT:
        return {
          className: styles.assessment,
          icon: '✅',
          title: `Assessment: ${title || 'Assessment Activity'}`
        };
      case ExerciseType.CHECKLIST:
        return {
          className: styles.checklist,
          icon: '📋',
          title: `Checklist: ${title || 'Implementation Checklist'}`
        };
      default:
        return {
          className: styles.exercise,
          icon: '✍️',
          title: `Exercise: ${title || 'Exercise'}`
        };
    }
  };

  const config = getExerciseConfig();

  return (
    <div className={clsx('exercise-container', config.className, styles[`difficulty-${difficulty}`])}>
      <div className={styles.header}>
        <span className={styles.icon}>{config.icon}</span>
        <h3 className={styles.title}>{config.title}</h3>
      </div>
      <div className={styles.content}>
        {children}
      </div>
    </div>
  );
};

export default Exercise;