/**
 * OceanCard Component
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Reusable glassmorphism card with Ocean Sapphire styling
 * Tasks: T024-T026
 *
 * Variants: stats, chapter, highlight, default
 */

import React from 'react';
import type { OceanCardProps } from '../../../specs/003-ocean-sapphire-theme/contracts/OceanCard.interface';

const OceanCard: React.FC<OceanCardProps> = ({
  variant = 'default',
  children,
  enableShimmer = true,
  className = '',
  onClick,
  width = 'auto',
  height = 'auto',
  padding,
}) => {
  // T025: Build CSS class names for glassmorphism styles
  const baseClass = 'ocean-card';
  const variantClass = variant !== 'default' ? `ocean-card--${variant}` : '';
  const shimmerClass = enableShimmer ? 'ocean-card--shimmer' : '';
  const clickableClass = onClick ? 'ocean-card--clickable' : '';

  const allClasses = [
    baseClass,
    variantClass,
    shimmerClass,
    clickableClass,
    className,
  ]
    .filter(Boolean)
    .join(' ');

  // T025: Inline styles for customization
  // For stats cards, enforce 3:2 aspect ratio
  const cardStyle: React.CSSProperties = {
    width: variant === 'stats' ? '100%' : width,
    height: variant === 'stats' ? 'auto' : height,
    aspectRatio: variant === 'stats' ? '3/2' : undefined,
    ...(padding && { padding }),
    display: variant === 'stats' ? 'flex' : undefined,
    flexDirection: variant === 'stats' ? 'column' : undefined,
    justifyContent: variant === 'stats' ? 'center' : undefined,
    alignItems: variant === 'stats' ? 'center' : undefined,
  };

  return (
    <div
      className={allClasses}
      style={cardStyle}
      onClick={onClick}
      role={onClick ? 'button' : undefined}
      tabIndex={onClick ? 0 : undefined}
    >
      {children}
    </div>
  );
};

export default OceanCard;
