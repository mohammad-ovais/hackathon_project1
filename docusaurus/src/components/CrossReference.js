import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './CrossReference.module.css';

const CrossReference = ({ to, title, type = "section", children }) => {
  return (
    <div className={clsx('cross-reference-container', styles.container)}>
      <div className={styles.content}>
        <span className={styles.refType}>{type.toUpperCase()}:</span>
        <Link to={to} className={styles.refLink}>
          {title}
        </Link>
        {children && <div className={styles.description}>{children}</div>}
      </div>
      <span className={styles.icon}>🔗</span>
    </div>
  );
};

export default CrossReference;