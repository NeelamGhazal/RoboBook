import React, { useState } from 'react';
import Link from '@docusaurus/Link';

/**
 * Custom Navbar Component for Landing Page
 * Feature: 004-landing-page-redesign
 * Constitution v1.4.0 §VIII - Navbar Requirements
 * Constitution v1.4.0 §XX - Theme Toggle Requirements
 *
 * Specifications:
 * - Height: 72px fixed
 * - Logo: 40×40px icon + "RoboBook" text (Orbitron 22px/700)
 * - Menu items: "Textbook | Blog" (17px, 32px gaps) - on left side
 * - Right controls: Language toggle (EN/UR), Theme toggle (28px, 24px spacing)
 * - Backdrop blur with glassmorphism
 */

interface NavbarProps {
  theme?: 'dark' | 'light';
  toggleTheme?: () => void;
}

const Navbar: React.FC<NavbarProps> = ({ theme = 'dark', toggleTheme }) => {
  const [language, setLanguage] = useState<'EN' | 'UR'>('EN');

  const toggleLanguage = () => {
    setLanguage(prev => prev === 'EN' ? 'UR' : 'EN');
  };

  return (
    <nav className="landing-navbar">
      <div className="navbar-container">
        {/* Left: Logo + Brand + Menu Items */}
        <div className="navbar-left">
          <Link to="/" className="navbar-logo-link">
            <span className="navbar-brand-text">RoboBook</span>
          </Link>

          <Link to="/docs/intro" className="navbar-menu-item">
            Textbook
          </Link>
          <span className="navbar-separator">|</span>
          <Link to="/blog" className="navbar-menu-item">
            Blog
          </Link>
        </div>

        {/* Right: Controls (Language, Theme) */}
        <div className="navbar-right">
          {/* Language Toggle (Visual Only - US5) */}
          <button
            className="navbar-control language-toggle"
            aria-label={`Switch to ${language === 'EN' ? 'Urdu' : 'English'} language`}
            onClick={toggleLanguage}
          >
            <span className="language-text">{language}</span>
          </button>

          {/* Theme Toggle (US3 - functional) */}
          <button
            className="navbar-control theme-toggle"
            aria-label={`Switch to ${theme === 'dark' ? 'light' : 'dark'} theme`}
            onClick={toggleTheme}
          >
            <span className="theme-icon">
              {theme === 'dark' ? '☀️' : '🌙'}
            </span>
          </button>
        </div>
      </div>
    </nav>
  );
};

export default Navbar;
