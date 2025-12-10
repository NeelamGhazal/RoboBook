import React from 'react';
import Link from '@docusaurus/Link';
import styles from './Hero.module.css';

/**
 * Hero component for the landing page
 * Displays the main heading, subheading, and CTA button for navigation to the textbook
 *
 * Features:
 * - Gradient heading (cyan to magenta)
 * - Neon glow hover effect on CTA button
 * - Pulse animation on button
 * - Keyboard accessible with focus indicators
 * - SPA navigation to /docs/intro
 */
export const Hero: React.FC = () => {
  return (
    <div className={styles.hero}>
      <h1 className={styles.heading}>
        Physical AI & Humanoid Robotics
      </h1>
      <p className={styles.subheading}>
        From Digital Intelligence to Embodied Systems
      </p>
      <Link
        to="/docs/intro"
        className={styles.ctaButton}
        aria-label="Navigate to textbook introduction"
      >
        Read Book
      </Link>
    </div>
  );
};
