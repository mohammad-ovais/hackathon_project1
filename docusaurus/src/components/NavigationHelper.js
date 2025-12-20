import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './NavigationHelper.module.css';

const NavigationHelper = ({ prev, next, title = "Continue Learning" }) => {
  return (
    <div className={clsx('navigation-helper-container', styles.container)}>
      <div className={styles.header}>
        <span className={styles.icon}>🧭</span>
        <h3 className={styles.title}>{title}</h3>
      </div>
      <div className={styles.navigationLinks}>
        {prev && (
          <div className={styles.navLink}>
            <span className={styles.navDirection}>← Previous</span>
            <Link to={prev.url} className={styles.navTitle}>
              {prev.title}
            </Link>
          </div>
        )}
        {next && (
          <div className={styles.navLink}>
            <span className={styles.navDirection}>Next →</span>
            <Link to={next.url} className={styles.navTitle}>
              {next.title}
            </Link>
          </div>
        )}
      </div>
    </div>
  );
};

export default NavigationHelper;