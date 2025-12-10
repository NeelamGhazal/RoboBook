import React from 'react';
import FeatureCard from './FeatureCard';

/**
 * Feature Cards Section Component
 * Feature: 004-landing-page-redesign
 * Constitution v1.4.0 §VIII - Feature Cards Requirements
 *
 * Specifications:
 * - Section heading: Georgia serif 2.8rem, centered
 * - Description: 1.1rem, max-width 900px, centered, mb-16
 * - Grid: 3 columns desktop, 2 columns tablet, 1 column mobile
 * - Max-width: 1200px, centered, 80px vertical padding
 * - Gaps: 24px desktop, 16px mobile
 */
const FeatureCards: React.FC = () => {
  const features = [
    {
      title: "AI/Spec-Driven Book Creation",
      description: "Write and publish a complete book using Docusaurus, Claude Code, and Spec-Kit Plus.",
      imageSrc: "/img/undraw_docusaurus_mountain.svg",
      imageAlt: "AI-driven book creation workflow illustration"
    },
    {
      title: "Integrated RAG Chatbot",
      description: "Build and embed an intelligent RAG chatbot using OpenAI Agents, FastAPI, Qdrant, and Neon Postgres.",
      imageSrc: "/img/undraw_docusaurus_react.svg",
      imageAlt: "RAG chatbot integration illustration"
    },
    {
      title: "Personalization & Translations",
      description: "Add dynamic personalization, user-based customization, and one-click Urdu translation to every chapter.",
      imageSrc: "/img/undraw_docusaurus_tree.svg",
      imageAlt: "Content personalization and translation illustration"
    }
  ];

  return (
    <section className="features-section">
      <div className="features-container">
        <h2 className="features-heading">
          What's Inside This Book?
        </h2>
        <p className="features-description">
          This book guides you step-by-step through building a complete Physical AI project—from writing a spec-driven book to embedding an intelligent RAG chatbot, personalizing content, and integrating humanoid robotics simulations.
        </p>
        <div className="features-grid">
          {features.map((feature, index) => (
            <FeatureCard
              key={index}
              title={feature.title}
              description={feature.description}
              imageSrc={feature.imageSrc}
              imageAlt={feature.imageAlt}
            />
          ))}
        </div>
      </div>
    </section>
  );
};

export default FeatureCards;