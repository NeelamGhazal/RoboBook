import React from 'react';
import { motion } from 'framer-motion';
import styles from './StatCard.module.css';

/**
 * StatCard component for displaying textbook statistics
 *
 * Features:
 * - Glass-morphism effect (backdrop-filter: blur(10px))
 * - Hover effects: 10px lift + cyan glow
 * - Framer Motion animations
 * - Light/dark mode support
 *
 * @param title - Statistic label (e.g., "Modules", "Chapters")
 * @param value - Statistic value (e.g., "4", "21", "50+")
 */
interface StatCardProps {
  title: string;
  value: string;
}

export const StatCard: React.FC<StatCardProps> = ({ title, value }) => {
  return (
    <motion.div
      className={styles.card}
      whileHover={{ y: -10, boxShadow: '0 0 20px rgba(0, 240, 255, 0.6)' }}
      transition={{ duration: 0.3 }}
    >
      <h3 className={styles.value}>{value}</h3>
      <p className={styles.title}>{title}</p>
    </motion.div>
  );
};
