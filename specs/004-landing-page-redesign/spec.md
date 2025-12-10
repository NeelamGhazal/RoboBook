# Feature Specification: RoboBook Landing Page Redesign

**Feature Branch**: `004-landing-page-redesign`
**Created**: 2025-12-10
**Status**: Draft
**Input**: User description: "Redesign the RoboBook landing page according to the updated Constitution. Target goal: Build a futuristic, elegant landing page with precise sizing as defined in Constitution. Apply Orbitron font ONLY on landing page + navbar (docs/blog remain default). Replace animation with a professional, clean, 3D half-body robot image. Match hero layout exactly like reference image: left text, right robot. Navbar with RoboBook logo, EN/UR toggle, theme toggle, perfect sizing. Add 3 feature cards with glassmorphism and precise dimensions."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - View Professional Landing Page (Priority: P1)

As a visitor arriving at the RoboBook website, I want to immediately see a professional, futuristic landing page with clear branding and content overview, so that I understand what the platform offers and feel confident exploring further.

**Why this priority**: This is the core value proposition - the first impression that determines whether visitors continue engaging with the platform.

**Independent Test**: Can be fully tested by navigating to the homepage and verifying all visual elements (hero section, robot image, layout, typography) match the Constitution specifications and deliver immediate understanding of the platform's purpose.

**Acceptance Scenarios**:

1. **Given** a visitor navigates to the homepage, **When** the page loads, **Then** they see a hero section with "Physical AI & Humanoid Robotics" heading at 4.5rem, a descriptive paragraph at 1.3rem, and a "Start Learning" button at 200×60px
2. **Given** the page is loaded on desktop (≥1024px), **When** the visitor views the hero section, **Then** they see a 50/50 layout with text on the left and a professional 3D half-body robot image (520–580px width) on the right
3. **Given** the page is loaded on tablet (768px–1023px), **When** the visitor views the hero section, **Then** they see a 60/40 layout with text taking 60% and robot image 40%
4. **Given** the page is loaded on mobile (<768px), **When** the visitor views the hero section, **Then** they see a stacked layout with text above and robot image below, both at 100% width

---

### User Story 2 - Experience Consistent Branding (Priority: P1)

As a visitor exploring the RoboBook website, I want to see consistent Ocean Sapphire branding throughout the landing page, so that I perceive the platform as professional and cohesive.

**Why this priority**: Brand consistency is critical for establishing credibility and trust, directly impacting conversion rates.

**Independent Test**: Can be fully tested by inspecting all typography, colors, spacing, and visual elements against Constitution specifications (Section VII Brand Consistency, Section VIII Mandatory Page Structure).

**Acceptance Scenarios**:

1. **Given** the landing page is loaded, **When** the visitor inspects the navbar, **Then** they see a fixed-top navbar at 72px height with RoboBook logo (40×40px icon, 22px Orbitron text), menu items at 17px with 32px gaps, and right controls at 28px icons with 24px spacing
2. **Given** the visitor views any heading on the landing page, **When** they inspect the typography, **Then** all headings use Orbitron font (hero heading 4.5rem/700, section headings 2.8rem, card titles 1.5rem/700), while paragraphs use Georgia serif (hero 1.3rem, cards 1rem)
3. **Given** the visitor views the feature cards section, **When** they inspect the cards, **Then** they see 3 cards with glassmorphism styling (rgba(0, 50, 100, 0.3) background, backdrop-filter blur, 12px border-radius), 220px minimum height, 24px padding, and 40/60 image/text split
4. **Given** the landing page is loaded, **When** the visitor inspects color usage, **Then** all colors match Ocean Sapphire palette (background #001529, text #b8d4ff, accent cyan #0096ff, borders rgba(0, 150, 255, 0.3))

---

### User Story 3 - Switch Between Light and Dark Themes (Priority: P2)

As a visitor with accessibility preferences or visual preferences, I want to toggle between light and dark themes, so that I can view the content in my preferred color scheme.

**Why this priority**: Accessibility and user preference support enhance user experience and inclusivity, but are secondary to core content presentation.

**Independent Test**: Can be fully tested by clicking the theme toggle in the navbar and verifying that colors transition smoothly (200ms ease-in-out) between dark and light palettes while layout/spacing/typography remain unchanged.

**Acceptance Scenarios**:

1. **Given** the landing page loads with default dark theme, **When** the visitor clicks the theme toggle in the navbar, **Then** the page smoothly transitions (200ms ease-in-out) to light theme (background #f3f8ff, text #001529, cards rgba(255, 255, 255, 0.6))
2. **Given** the page is in light theme, **When** the visitor clicks the theme toggle again, **Then** the page smoothly transitions back to dark theme (background #001529, text #b8d4ff, cards rgba(0, 50, 100, 0.3))
3. **Given** the page is in either theme, **When** the visitor inspects contrast ratios, **Then** all text meets WCAG AA standards (contrast ratio ≥4.5:1)
4. **Given** the theme is toggled, **When** the visitor observes the transition, **Then** only colors change while layout, spacing, typography, and animations remain identical

---

### User Story 4 - Interact with Feature Cards (Priority: P3)

As a visitor exploring the platform's capabilities, I want to see and interact with feature cards that showcase what's inside the book, so that I understand the specific value propositions.

**Why this priority**: Feature cards provide detailed value communication but are supplementary to the primary hero section messaging.

**Independent Test**: Can be fully tested by viewing the feature cards section and verifying layout (1200px max-width, 80px vertical padding, 24px desktop/16px mobile gaps), card styling (220px min height, 3:2 ratio images, shimmer effect on hover), and responsive behavior (3 columns desktop, 2 columns tablet, 1 column mobile).

**Acceptance Scenarios**:

1. **Given** a visitor scrolls to the features section, **When** they view the cards on desktop (≥1024px), **Then** they see 3 cards in a row with 24px gaps, each card displaying a 3:2 ratio image (40% height) and text content (60% height)
2. **Given** a visitor hovers over a feature card, **When** the cursor is over the card, **Then** the card shows a subtle shimmer effect (0.3s ease transition)
3. **Given** the page is loaded on tablet (768px–1023px), **When** the visitor views the features section, **Then** they see 2 cards per row with 16px gaps
4. **Given** the page is loaded on mobile (<768px), **When** the visitor views the features section, **Then** they see 1 card per row stacked vertically with 16px gaps

---

### User Story 5 - Navigate Using Language Toggle (Priority: P3)

As a visitor who speaks Urdu, I want to see a language toggle (EN/UR) in the navbar, so that I know the platform supports my language (even if functionality is not yet implemented).

**Why this priority**: This is a visual placeholder for future functionality, important for user perception but not functionally active in this phase.

**Independent Test**: Can be fully tested by verifying the EN/UR toggle appears in the navbar with proper styling (28px icons, 24px spacing) and visual-only functionality (no actual translation).

**Acceptance Scenarios**:

1. **Given** the landing page loads, **When** the visitor inspects the navbar, **Then** they see an EN/UR language toggle positioned between the RoboBook logo and the theme toggle
2. **Given** the visitor clicks the language toggle, **When** the toggle is activated, **Then** the visual state changes but no actual translation occurs (visual-only placeholder)
3. **Given** the navbar is displayed on mobile, **When** the visitor views the controls, **Then** the language toggle remains visible and accessible at 28px icon size

---

### Edge Cases

- **What happens when the viewport width is exactly 768px?** The layout should transition smoothly between mobile and tablet breakpoints using CSS media queries with clear boundaries (mobile <768px, tablet 768px–1023px, desktop ≥1024px).
- **What happens if the robot image fails to load?** The layout should maintain its structure with an alt text placeholder, ensuring no layout shift occurs (CLS = 0).
- **What happens when JavaScript is disabled?** The page should load with default dark theme and all static content should remain accessible, though the theme toggle will not be functional.
- **What happens when users have reduced motion preferences?** The 200ms theme transitions should respect `prefers-reduced-motion` CSS media query and apply instant color changes instead.
- **What happens on ultra-wide displays (>1920px)?** The hero section and feature cards should respect max-width constraints (hero max-width 1400px, features max-width 1200px) and remain centered.

## Requirements *(mandatory)*

### Functional Requirements

#### Hero Section Requirements
- **FR-001**: The hero section MUST display a heading "Physical AI & Humanoid Robotics" in Orbitron font at 4.5rem (72px) with 700 weight and line-height 1.2
- **FR-002**: The hero section MUST display a descriptive paragraph in Georgia serif font at 1.3rem (20px) explaining the platform's purpose
- **FR-003**: The hero section MUST include a "Start Learning" button at 200×60px with Orbitron font at 1.2rem (18px) and 700 weight
- **FR-004**: The hero section MUST display a professional 3D half-body robot image at 520–580px width with auto height in WebP format
- **FR-005**: The hero section MUST implement responsive layouts: 50/50 desktop (≥1024px), 60/40 tablet (768px–1023px), stacked mobile (<768px)
- **FR-006**: The hero section MUST include 100px top padding, 24px side padding, 20px heading-paragraph gap, and 28px paragraph-button gap

#### Navbar Requirements
- **FR-007**: The navbar MUST be fixed-top with 72px height, backdrop-filter blur, and z-index appropriate for staying above page content
- **FR-008**: The navbar MUST display the RoboBook logo with 40×40px icon and "RoboBook" text in Orbitron at 22px with 700 weight
- **FR-009**: The navbar MUST include menu items ("Home", "Docs", "Blog", "GitHub") at 17px font-size with 32px horizontal spacing
- **FR-010**: The navbar MUST include right controls (language toggle EN/UR, theme toggle, GitHub link) at 28px icon size with 24px spacing between controls

#### Feature Cards Requirements
- **FR-011**: The feature cards section MUST display exactly 3 cards showcasing "AI/Spec-Driven Book Creation", "Integrated RAG Chatbot", and "Personalization & Translations"
- **FR-012**: Each feature card MUST have a minimum height of 220px, 12px border-radius, 24px padding, and glassmorphism styling (rgba(0, 50, 100, 0.3) background with backdrop-filter blur)
- **FR-013**: Each feature card MUST display a 3:2 ratio image occupying 40% of card height with the remaining 60% for text content
- **FR-014**: Feature cards MUST implement 40/60 image/text split with images optimized to WebP format (max 50KB each)
- **FR-015**: Feature cards MUST include card titles in Orbitron font at 1.5rem (24px) with 700 weight
- **FR-016**: Feature cards MUST include card descriptions in Georgia serif font at 1rem (16px)
- **FR-017**: Feature cards MUST display in responsive layouts: 3 columns desktop (24px gaps), 2 columns tablet (16px gaps), 1 column mobile (16px gaps)
- **FR-018**: The feature cards section MUST have 1200px max-width, 80px vertical padding, and be centered on the page
- **FR-019**: Feature cards MUST implement a subtle shimmer effect on hover with 0.3s ease transition
- **FR-020**: Feature card images MUST be lazy-loaded to improve initial page load performance

#### Theme Toggle Requirements
- **FR-021**: The landing page MUST include a light/dark theme toggle control located in the navbar
- **FR-022**: The theme toggle MUST default to dark theme (Ocean Sapphire palette: background #001529, text #b8d4ff, accent #0096ff, borders rgba(0, 150, 255, 0.3), cards rgba(0, 50, 100, 0.3))
- **FR-023**: The theme toggle MUST switch to light theme (background #f3f8ff, text #001529, accent #0096ff, borders rgba(0, 150, 255, 0.3), cards rgba(255, 255, 255, 0.6))
- **FR-024**: The theme toggle MUST implement React state management (no localStorage) for theme switching
- **FR-025**: The theme toggle MUST implement 200ms ease-in-out transitions for all color changes
- **FR-026**: The theme toggle MUST maintain WCAG AA contrast compliance (contrast ratio ≥4.5:1) in both light and dark modes
- **FR-027**: The theme toggle MUST only change colors while keeping layout, spacing, typography, and animations identical

#### Language Toggle Requirements
- **FR-028**: The navbar MUST include an EN/UR language toggle control positioned between the logo and theme toggle
- **FR-029**: The language toggle MUST be visual-only (no functional translation implementation in this phase)
- **FR-030**: The language toggle MUST maintain consistent styling with other navbar controls (28px icons, 24px spacing)

#### Typography Rules
- **FR-031**: Orbitron font MUST be used exclusively for: navbar elements, landing page headings, and landing page CTAs
- **FR-032**: Orbitron font MUST NOT be used in: docs content, blog content, sidebar navigation, or chapter content
- **FR-033**: Georgia serif font MUST be used for all paragraph text on the landing page

#### File Modification Restrictions
- **FR-034**: Changes MUST be limited exclusively to: src/pages/index.tsx, src/components/Hero/*, src/components/FeatureCards/*, src/css/custom.css
- **FR-035**: Changes MUST NOT be made to: routing configuration, docs/blog content or styling, Docusaurus config (except navbar items), animation code files
- **FR-036**: The implementation MUST NOT create new pages, backend logic, translation systems, chatbot functionality, or modify routing

### Key Entities

- **Hero Section**: The primary landing page section containing the main value proposition heading, descriptive paragraph, call-to-action button, and robot image. Displays in responsive layouts (50/50 desktop, 60/40 tablet, stacked mobile).

- **Feature Card**: A visual card component showcasing one platform capability with a 3:2 ratio image (40% height), title (Orbitron 1.5rem), and description (Georgia 1rem). Implements glassmorphism styling with hover shimmer effect.

- **Theme State**: React state managing the current theme mode (light or dark), controlling the application of color palettes while preserving all other design aspects. Transitions smoothly (200ms ease-in-out) between modes.

- **Navbar**: Fixed-top navigation bar (72px height) with RoboBook branding, menu items, language toggle (EN/UR), theme toggle, and GitHub link. Uses backdrop-filter blur and maintains consistent spacing.

- **Robot Image**: Professional 3D half-body robot image (520–580px width, WebP format, max 200KB) displayed in the hero section's right column (desktop/tablet) or below text (mobile).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The landing page loads within 3 seconds on 3G network connections with all critical content visible (hero section, navbar, first feature card)
- **SC-002**: All typography matches Constitution specifications exactly (hero heading 4.5rem, hero paragraph 1.3rem, button text 1.2rem, section heading 2.8rem, card titles 1.5rem, card text 1rem)
- **SC-003**: All text elements use specified fonts precisely (Orbitron for navbar/headings/CTAs, Georgia serif for paragraphs)
- **SC-004**: The navbar maintains exactly 72px height across all viewport sizes with perfect spacing (40×40px icon, 22px logo text, 17px menu items, 32px gaps, 28px control icons, 24px spacing)
- **SC-005**: The hero section implements responsive layouts correctly (50/50 desktop ≥1024px, 60/40 tablet 768px–1023px, stacked mobile <768px)
- **SC-006**: The theme toggle switches between light and dark modes within 200ms with smooth ease-in-out transitions
- **SC-007**: Both light and dark themes maintain WCAG AA contrast compliance (contrast ratios ≥4.5:1) verified using automated accessibility testing tools
- **SC-008**: Feature cards display in correct responsive grids (3 columns desktop with 24px gaps, 2 columns tablet with 16px gaps, 1 column mobile with 16px gaps)
- **SC-009**: The page achieves zero cumulative layout shift (CLS = 0) with no visual jumping during load or theme transitions
- **SC-010**: The robot image and all feature card images are optimized to WebP format (robot max 200KB, feature cards max 50KB each) and lazy-loaded
- **SC-011**: The landing page renders with no console errors in Chrome, Firefox, and Safari on both desktop and mobile viewports
- **SC-012**: Orbitron font is applied exclusively to landing page elements (navbar, headings, CTAs) and does not affect docs or blog content
- **SC-013**: The landing page maintains visual consistency with Ocean Sapphire branding (background #001529, text #b8d4ff, accent #0096ff) in dark mode
- **SC-014**: All file modifications are limited to the allowed scope (index.tsx, Hero/*, FeatureCards/*, custom.css) with no changes to routing, docs, blog, or Docusaurus config

## Scope *(mandatory)*

### In Scope

- Redesigning the landing page hero section with professional robot image and precise typography
- Implementing fixed-top navbar with RoboBook branding, menu items, and control icons at exact dimensions
- Creating 3 feature cards with glassmorphism styling, 3:2 ratio images, and hover shimmer effects
- Adding light/dark theme toggle with React state and 200ms smooth transitions
- Adding visual-only EN/UR language toggle placeholder in navbar
- Implementing responsive layouts for desktop (≥1024px), tablet (768px–1023px), and mobile (<768px)
- Applying Orbitron font exclusively to landing page navbar, headings, and CTAs
- Optimizing all images to WebP format with specified size limits and lazy loading
- Ensuring WCAG AA contrast compliance in both light and dark themes
- Maintaining zero cumulative layout shift (CLS = 0) throughout the page

### Out of Scope

- Functional translation system (language toggle is visual-only placeholder)
- New pages, routes, or navigation changes
- Backend APIs or server-side logic
- Chatbot implementation or integration
- Modifications to docs or blog content, styling, or typography
- Changes to Docusaurus configuration beyond navbar items
- Animation code or interactive visualizations
- Database or data persistence layer
- User authentication or session management
- Theme preference persistence (no localStorage, state-only)

### Dependencies

- Docusaurus 3.x framework for site structure
- React 18+ for component implementation and theme state management
- Professional 3D robot image asset in WebP format (520–580px width, max 200KB)
- Feature card image assets in WebP format (3:2 ratio, max 50KB each)
- Orbitron font family (Google Fonts or local)
- Georgia serif font (system default)
- Constitution v1.4.0 specifications for all design requirements

### Assumptions

- The existing Docusaurus site structure remains unchanged (only landing page modified)
- The provided robot image meets quality and dimension requirements (520–580px width, WebP, max 200KB)
- Feature card images are available in 3:2 ratio at appropriate sizes
- Orbitron font is already configured or will be loaded from Google Fonts
- Modern browsers with CSS backdrop-filter support are the target (Chrome, Firefox, Safari, Edge)
- The Constitution v1.4.0 specifications are final and will not change during implementation

## Non-Functional Requirements

### Performance
- Page must load within 3 seconds on 3G network connections
- Images must be optimized (robot max 200KB, cards max 50KB each) and lazy-loaded
- Zero cumulative layout shift (CLS = 0) throughout page load and theme transitions
- Theme transitions must complete within 200ms (ease-in-out)
- Hover effects on feature cards must respond within 0.3s (ease transition)

### Accessibility
- WCAG AA contrast compliance (≥4.5:1) in both light and dark themes
- Semantic HTML structure for screen reader compatibility
- Keyboard navigation support for theme toggle and language toggle
- Alt text for all images (robot image and feature card images)
- Respect `prefers-reduced-motion` for users who prefer reduced animations

### Browser Compatibility
- Chrome (latest 2 versions)
- Firefox (latest 2 versions)
- Safari (latest 2 versions)
- Edge (latest 2 versions)
- Mobile browsers: iOS Safari, Chrome Android

### Responsive Design
- Desktop (≥1024px): 50/50 hero layout, 3-column feature cards (24px gaps)
- Tablet (768px–1023px): 60/40 hero layout, 2-column feature cards (16px gaps)
- Mobile (<768px): Stacked hero layout, 1-column feature cards (16px gaps)
- Navbar maintains 72px height across all viewports

### Code Quality
- React components must be functional components with TypeScript types
- CSS must be organized in custom.css with clear section comments
- No inline styles except for specific Orbitron font-family applications
- Code must follow existing Docusaurus project structure and conventions
- No console errors or warnings in production build

### Security
- No external API calls or data transmission (static site only)
- No user input collection or storage
- No third-party tracking scripts or analytics beyond Docusaurus defaults
- No localStorage usage (theme state is React-only, not persisted)
