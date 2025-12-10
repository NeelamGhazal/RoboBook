import React from 'react';
import Link from '@docusaurus/Link';

/**
 * Hero Section Component
 * Feature: 004-landing-page-redesign
 * Constitution v1.4.0 §VIII - Hero Section Requirements
 *
 * Layout:
 * - Desktop (≥1024px): 50/50 text/image split
 * - Tablet (768-1023px): 60/40 text/image split
 * - Mobile (<768px): Stacked layout (text above, image below)
 *
 * Typography:
 * - Heading: Orbitron 4.5rem/700 (responsive: 2.5rem mobile, 3rem tablet, 4.5rem desktop)
 * - Paragraph: Georgia serif 1.3rem (responsive: 1.3rem mobile, 1.5rem tablet, 1.8rem desktop)
 * - Button: 200×60px, Orbitron 1.2rem/700
 *
 * Image:
 * - Robot: 520-580px width, WebP format, lazy-loaded, <200KB
 */
const Hero: React.FC = () => {
  return (
    <section className="hero-section">
      <div className="hero-container">
        {/* Left: Text Content (50% desktop, 60% tablet, 100% mobile) */}
        <div className="hero-content">
          <h1 className="hero-heading">
            Physical AI & Humanoid Robotics
          </h1>
          <p className="hero-paragraph">
            Learn to control physical androids using ROS 2, advanced vision systems,
            large language models, and reinforcement learning with this free, open-source textbook.
          </p>
          <Link to="/docs/intro" className="hero-cta-button">
            Start Learning
          </Link>
        </div>

        {/* Right: Robot Image (50% desktop, 40% tablet, 100% mobile) */}
        <div className="hero-image">
          <img
            src="/img/hero.jpg"
            alt="Professional 3D half-body robot illustration representing humanoid robotics and AI"
            className="robot-image"
            loading="lazy"
            width="550"
            height="auto"
          />
        </div>
      </div>
    </section>
  );
};

export default Hero;