/**
 * Ocean Card Component Interface
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Reusable glassmorphism card with shimmer animation
 * Location: website/src/components/OceanCard.tsx
 * Variants: Stats Card, Chapter Card, Highlight Box
 */

import type { ReactNode } from 'react';

/**
 * Ocean Card Component Props
 *
 * Configurable glassmorphism card with Ocean Sapphire styling
 */
export interface OceanCardProps {
  /**
   * Card variant type
   * @default 'default'
   * @description
   * - 'stats': Landing page stats cards (4 Modules, 21 Chapters, etc.)
   * - 'chapter': Book reader chapter cards
   * - 'highlight': Markdown highlight boxes with 💎 icon
   * - 'default': Generic glassmorphism card
   */
  variant?: 'stats' | 'chapter' | 'highlight' | 'default';

  /**
   * Card content (React children)
   * @required
   */
  children: ReactNode;

  /**
   * Enable shimmer animation on hover
   * @default true
   * @description 3-second infinite gradient sweep from left to right
   */
  enableShimmer?: boolean;

  /**
   * Custom CSS class name
   * @optional
   * @example 'landing-stats-card'
   */
  className?: string;

  /**
   * Click handler (for interactive cards)
   * @optional
   * @description Makes card clickable with hover effects
   */
  onClick?: () => void;

  /**
   * Card width
   * @default 'auto' (full-width for chapter/highlight, fixed for stats)
   * @example '200px', '100%', 'auto'
   */
  width?: string;

  /**
   * Card height
   * @default 'auto'
   * @example '150px', '100%', 'auto'
   */
  height?: string;

  /**
   * Custom padding (overrides default 2rem)
   * @optional
   * @example '1.5rem', '3rem'
   */
  padding?: string;
}

/**
 * Stats Card Specific Props
 *
 * For landing page stats cards variant
 */
export interface StatsCardProps extends Omit<OceanCardProps, 'variant' | 'children'> {
  /**
   * Large number display (e.g., "4", "21")
   * @example "4"
   */
  number: string;

  /**
   * Label text below number (e.g., "Modules", "Chapters")
   * @example "Modules"
   */
  label: string;

  /**
   * Optional icon above number
   * @optional
   * @example "📚", "🤖"
   */
  icon?: string;
}

/**
 * Chapter Card Specific Props
 *
 * For book reader chapter cards variant
 */
export interface ChapterCardProps extends Omit<OceanCardProps, 'variant' | 'children'> {
  /**
   * Chapter title
   * @example "Chapter 1: ROS 2 Nodes Architecture"
   */
  title: string;

  /**
   * Chapter description/summary
   * @example "Learn how to create and manage ROS 2 nodes..."
   */
  description: string;

  /**
   * Chapter badges (e.g., "Beginner", "30 min")
   * @optional
   */
  badges?: string[];

  /**
   * Chapter URL for navigation
   * @example "/docs/module-1-ros2/chapter-1-nodes-architecture"
   */
  href: string;
}

/**
 * Highlight Box Specific Props
 *
 * For markdown content highlight boxes variant
 */
export interface HighlightBoxProps extends Omit<OceanCardProps, 'variant'> {
  /**
   * Highlight box title (optional)
   * @optional
   * @example "Key Insight"
   */
  title?: string;

  /**
   * Icon to display (default: 💎)
   * @default "💎"
   */
  icon?: string;

  /**
   * Highlight box type for semantic meaning
   * @default 'info'
   * @description
   * - 'info': Blue glow (default)
   * - 'warning': Yellow glow (future enhancement)
   * - 'success': Green glow (future enhancement)
   */
  type?: 'info' | 'warning' | 'success';
}

/**
 * Ocean Card CSS Classes
 *
 * BEM-style class naming convention
 */
export const OceanCardClasses = {
  BASE: 'ocean-card',
  VARIANT_STATS: 'ocean-card--stats',
  VARIANT_CHAPTER: 'ocean-card--chapter',
  VARIANT_HIGHLIGHT: 'ocean-card--highlight',
  SHIMMER: 'ocean-card--shimmer',
  CLICKABLE: 'ocean-card--clickable',
  NUMBER: 'ocean-card__number',
  LABEL: 'ocean-card__label',
  ICON: 'ocean-card__icon',
  TITLE: 'ocean-card__title',
  DESCRIPTION: 'ocean-card__description',
  BADGES: 'ocean-card__badges',
  BADGE: 'ocean-card__badge',
} as const;

/**
 * Glassmorphism CSS Properties
 *
 * Core styling values for Ocean Sapphire glassmorphism effect
 */
export const GlassmorphismStyles = {
  BACKGROUND_WITH_BLUR: 'rgba(0, 50, 100, 0.3)',
  BACKGROUND_FALLBACK: 'rgba(0, 50, 100, 0.7)',
  BACKDROP_FILTER: 'blur(10px)',
  BORDER: '1px solid rgba(0, 150, 255, 0.3)',
  BORDER_RADIUS: '12px',
  BOX_SHADOW: '0 10px 40px rgba(0, 0, 0, 0.5)',
  HOVER_SHADOW: '0 10px 50px rgba(0, 150, 255, 0.3)',
  PADDING: '2rem',
} as const;

/**
 * Shimmer Animation Configuration
 *
 * Gradient sweep animation for card hover effects
 */
export const ShimmerAnimation = {
  DURATION: '3s',
  TIMING_FUNCTION: 'linear',
  ITERATION_COUNT: 'infinite',
  GRADIENT: 'linear-gradient(90deg, rgba(0, 50, 100, 0.3) 0%, rgba(0, 150, 255, 0.2) 50%, rgba(0, 50, 100, 0.3) 100%)',
  BACKGROUND_SIZE: '200% 100%',
} as const;

/**
 * Usage Examples:
 *
 * 1. Stats Card (Landing Page)
 * ```tsx
 * <StatsCard
 *   number="4"
 *   label="Modules"
 *   icon="📚"
 *   enableShimmer={true}
 * />
 * ```
 *
 * 2. Chapter Card (Book Reader)
 * ```tsx
 * <ChapterCard
 *   title="Chapter 1: ROS 2 Nodes Architecture"
 *   description="Learn how to create and manage ROS 2 nodes..."
 *   badges={["Beginner", "30 min"]}
 *   href="/docs/module-1-ros2/chapter-1-nodes-architecture"
 *   enableShimmer={true}
 * />
 * ```
 *
 * 3. Highlight Box (Markdown)
 * ```tsx
 * <HighlightBox icon="💎" title="Key Insight">
 *   Ocean Sapphire glassmorphism creates depth without heavy graphics.
 * </HighlightBox>
 * ```
 *
 * Validation Checklist:
 * - [ ] Glassmorphism uses @supports for backdrop-filter fallback
 * - [ ] Hover effects respect prefers-reduced-motion
 * - [ ] All variants work on mobile (touch-friendly, no hover-only interactions)
 * - [ ] WCAG AAA contrast maintained (text on glassmorphism background)
 * - [ ] Shimmer animation uses GPU-accelerated transform properties
 */
