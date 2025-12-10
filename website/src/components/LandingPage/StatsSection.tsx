import React from 'react';
import { motion, useAnimation, useInView } from 'framer-motion';
import { StatCard } from './StatCard';
import styles from './StatsSection.module.css';

/**
 * StatsSection component displaying textbook statistics
 *
 * Features:
 * - 4 stat cards with glass-morphism
 * - Staggered fade-in animation (100ms stagger)
 * - Scroll detection with useInView hook
 * - Responsive grid layout
 *
 * Stats displayed:
 * - 4 Modules
 * - 21 Chapters
 * - 50+ Code Examples
 * - AI-Powered Chatbot (✓)
 */

const stats = [
  { title: 'Modules', value: '4' },
  { title: 'Chapters', value: '21' },
  { title: 'Code Examples', value: '50+' },
  { title: 'AI-Powered Chatbot', value: '✓' }
];

export const StatsSection: React.FC = () => {
  const ref = React.useRef(null);
  const isInView = useInView(ref, { once: true, amount: 0.3 });
  const controls = useAnimation();

  React.useEffect(() => {
    if (isInView) {
      controls.start('visible');
    }
  }, [isInView, controls]);

  return (
    <motion.section
      ref={ref}
      className={styles.container}
      initial="hidden"
      animate={controls}
      variants={{
        hidden: { opacity: 0 },
        visible: {
          opacity: 1,
          transition: { staggerChildren: 0.1 }
        }
      }}
    >
      {stats.map((stat, index) => (
        <motion.div
          key={index}
          variants={{
            hidden: { opacity: 0, y: 20 },
            visible: { opacity: 1, y: 0 }
          }}
        >
          <StatCard title={stat.title} value={stat.value} />
        </motion.div>
      ))}
    </motion.section>
  );
};
