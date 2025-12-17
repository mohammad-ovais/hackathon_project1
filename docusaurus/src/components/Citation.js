import React from 'react';
import clsx from 'clsx';
import styles from './Citation.module.css';

const Citation = ({ id, authors, year, title, source, url, children }) => {
  return (
    <div className={clsx('citation-container', styles.container)}>
      <div className={styles.citationContent}>
        <div className={styles.citationText}>
          <span className={styles.citationId}>[{id}]</span> {authors} ({year}). {title}. {source}.{url && ` Available at: `}
          {url && <a href={url} className={styles.urlLink} target="_blank" rel="noopener noreferrer">{url}</a>}
        </div>
        {children && <div className={styles.description}>{children}</div>}
      </div>
      <span className={styles.icon}>📚</span>
    </div>
  );
};

export default Citation;