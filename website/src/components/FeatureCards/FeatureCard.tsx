import React from 'react';

/**
 * Individual Feature Card Component
 * Feature: 004-landing-page-redesign
 * Constitution v1.4.0 §VIII - Feature Card Requirements
 *
 * Specifications:
 * - Glassmorphism: backdrop-filter blur(10px), rgba background
 * - 40/60 image/text split (flexbox row)
 * - Image: 40% width, min 180px, 3:2 ratio, lazy-loaded
 * - Text: 60% width, title Orbitron 1.5rem/700, description Georgia 1rem
 * - Min-height: 220px, padding: 24px, border-radius: 12px
 * - Hover: shimmer effect (3s infinite)
 */

interface FeatureCardProps {
  title: string;
  description: string;
  imageSrc: string;
  imageAlt: string;
}

const FeatureCard: React.FC<FeatureCardProps> = ({ title, description, imageSrc, imageAlt }) => {
  return (
    <div className="feature-card">
      {/* Image Section (40%) */}
      <div className="feature-card-image">
        <img
          src={imageSrc}
          alt={imageAlt}
          loading="lazy"
          width="240"
          height="160"
          style={{ aspectRatio: '3 / 2' }}
        />
      </div>

      {/* Content Section (60%) */}
      <div className="feature-card-content">
        <h3 className="feature-card-title">
          {title}
        </h3>
        <p className="feature-card-description">
          {description}
        </p>
      </div>
    </div>
  );
};

export default FeatureCard;