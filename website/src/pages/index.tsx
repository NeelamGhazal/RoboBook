/**
 * Landing Page - Constitution v1.4.0
 *
 * Features:
 * - 003-ocean-sapphire-theme: Ocean Sapphire design system
 * - 004-landing-page-redesign: Constitution-compliant layout
 *
 * Components:
 * - Custom Navbar (72px, fixed, glassmorphism)
 * - Hero Section (50/50 desktop, 60/40 tablet, stacked mobile)
 * - Feature Cards (glassmorphism, responsive grid)
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Navbar from '../components/Navbar/Navbar';
import Hero from '../components/Hero/Hero';
import FeatureCards from '../components/FeatureCards/FeatureCards';

export default function LandingPage(): JSX.Element {
  const [theme, setTheme] = useState<'dark' | 'light'>('dark');

  useEffect(() => {
    // Set the theme attribute on the document element
    document.documentElement.setAttribute('data-theme', theme);
  }, [theme]);

  const toggleTheme = () => {
    setTheme(prevTheme => prevTheme === 'dark' ? 'light' : 'dark');
  };

  return (
    <Layout
      title="RoboBook - Physical AI & Humanoid Robotics"
      description="Interactive textbook for Physical AI and Humanoid Robotics"
      noFooter={false}
    >
      {/* Custom Navbar with theme toggle - Constitution §VIII, §XX */}
      <Navbar theme={theme} toggleTheme={toggleTheme} />

      <main style={{ marginTop: '72px' }}> {/* Offset for fixed navbar */}
        {/* Hero Section - Constitution §VIII */}
        <Hero />

        {/* Feature Cards Section - Constitution §VIII */}
        <FeatureCards />
      </main>
    </Layout>
  );
}
