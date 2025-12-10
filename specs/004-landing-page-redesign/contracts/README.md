# API Contracts

**Feature**: 004-landing-page-redesign
**Date**: 2025-12-10

## Status: Not Applicable

This feature is **frontend-only** with no backend APIs or external contracts.

## Rationale

The RoboBook landing page redesign is a static site implementation using Docusaurus and React. Per Constitution §XXII (Non-Negotiable Restrictions), the following are explicitly forbidden:

- Creating backend logic or API endpoints
- Adding external services or integrations
- Implementing translation systems with external APIs
- Adding chatbot functionality with external services

## Data Sources

All data for the landing page is **static and defined at build time**:

1. **Hero Content**: Hard-coded in `HeroContent` configuration object
2. **Feature Cards**: Static array of 3 `FeatureCard` objects
3. **Theme State**: React useState (no persistence, no API)
4. **Navbar Configuration**: Static `NavbarConfig` object
5. **Images**: Static assets in `website/static/img/`

## Component Interfaces

Instead of API contracts, this project uses **TypeScript interfaces** for component props and data structures. See [data-model.md](../data-model.md) for complete interface definitions:

- `ThemeState`
- `HeroContent`
- `FeatureCard`
- `FeatureCardsCollection`
- `NavbarConfig`
- Component props: `HeroProps`, `FeatureCardProps`, `FeatureCardsProps`, `NavbarProps`

## Future API Considerations

If future features require backend APIs (e.g., user authentication, content management, analytics), they should:

1. **Create separate feature branch** (e.g., `005-backend-api`)
2. **Update Constitution** to allow backend modifications for that specific feature
3. **Define OpenAPI/GraphQL schemas** in that feature's `contracts/` directory
4. **Maintain separation** between static frontend and dynamic backend

## Summary

**No API contracts exist for this feature.** All data flows are internal to the React application with no external dependencies, API calls, or backend services.
