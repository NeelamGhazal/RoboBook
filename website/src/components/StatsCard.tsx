/**
 * StatsCard Component
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Landing page stats cards with glassmorphism effects
 * Tasks: T027-T028
 *
 * Usage: Display stats like "4 Modules", "21 Chapters"
 */

import React from 'react';
import OceanCard from './OceanCard';
import type { StatsCardProps } from '../../../specs/003-ocean-sapphire-theme/contracts/OceanCard.interface';

const StatsCard: React.FC<StatsCardProps> = ({
  number,
  label,
  icon,
  enableShimmer = true,
  className = '',
  onClick,
  width,
  height,
  padding,
}) => {
  return (
    <OceanCard
      variant="stats"
      enableShimmer={enableShimmer}
      className={`stats-card ${className}`}
      onClick={onClick}
      width={width}
      height={height}
      padding={padding}
    >
      <div className="stats-card__content" style={{
        display: 'flex',
        flexDirection: 'row',
        alignItems: 'center',
        justifyContent: 'flex-start',
        width: '100%',
        padding: '20px'
      }}>
        {icon && (
          <div
            className="ocean-card__icon stats-card__icon"
            style={{
              fontSize: '60px', // 60×60px as requested
              width: '60px',
              minWidth: '60px',
              height: '60px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              marginRight: '15px',
            }}
          >
            {icon}
          </div>
        )}

        {/* T028: Number and label in a column to the right of the icon */}
        <div style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'flex-start',
          justifyContent: 'center',
          flex: 1
        }}>
          {/* T028: Number styled as 1.2em cyan (#00d4ff) */}
          <div
            className="ocean-card__number stats-card__number"
            style={{
              fontSize: '1.2em',
              fontWeight: 'bold',
              color: '#00d4ff',
              marginBottom: '5px',
              textShadow: '0 0 20px rgba(0, 212, 255, 0.5)',
            }}
          >
            {number}
          </div>

          {/* T028: Label styled as 0.95em caption color (#8bb3e0) */}
          <div
            className="ocean-card__label stats-card__label"
            style={{
              fontSize: '0.95em',
              color: '#8bb3e0',
              textTransform: 'uppercase',
              letterSpacing: '1px',
              fontWeight: 500,
            }}
          >
            {label}
          </div>
        </div>
      </div>
    </OceanCard>
  );
};

export default StatsCard;
