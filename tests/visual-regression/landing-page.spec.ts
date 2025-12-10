/**
 * Landing Page - Visual Regression Tests
 *
 * Feature: 003-ocean-sapphire-theme
 * Purpose: Playwright tests for Ocean Sapphire landing page visual consistency
 * Tasks: T037-T041
 *
 * Run: npx playwright test
 * Update snapshots: npx playwright test --update-snapshots
 */

import { test, expect } from '@playwright/test';

const BASE_URL = 'http://localhost:3000';

test.describe('Landing Page - Ocean Sapphire Theme', () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to landing page before each test
    await page.goto(BASE_URL);
  });

  // T038: Full-page screenshot baseline
  test('should match full-page screenshot baseline', async ({ page }) => {
    // Wait for page to fully load
    await page.waitForLoadState('networkidle');

    // Wait for Three.js scene to initialize (canvas element should be present)
    await page.waitForSelector('canvas', { timeout: 5000 });

    // Wait a bit for animations to stabilize
    await page.waitForTimeout(1000);

    // Take full-page screenshot
    await expect(page).toHaveScreenshot('landing-page-full.png', {
      fullPage: true,
      animations: 'disabled', // Disable animations for consistent screenshots
    });
  });

  // T039: Stats card hover state screenshot
  test('should capture stats card hover state', async ({ page }) => {
    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Wait for stats cards to be visible
    const statsCards = page.locator('.ocean-card');
    await statsCards.first().waitFor({ state: 'visible', timeout: 5000 });

    // Hover over the first stats card
    await statsCards.first().hover();

    // Wait for hover animation to trigger
    await page.waitForTimeout(500);

    // Take screenshot of the stats section
    const statsSection = page.locator('.landing-stats');
    await expect(statsSection).toHaveScreenshot('stats-cards-hover.png', {
      animations: 'disabled',
    });
  });

  // T040: Verify Three.js scene renders without errors
  test('should render Three.js scene without errors', async ({ page }) => {
    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Check for any console errors (specifically WebGL or Three.js errors)
    const errors: string[] = [];
    page.on('console', (msg) => {
      if (msg.type() === 'error') {
        errors.push(msg.text());
      }
    });

    // Wait for Three.js canvas to render
    const canvas = page.locator('canvas');
    await expect(canvas).toBeVisible({ timeout: 5000 });

    // Verify canvas has non-zero dimensions
    const canvasBox = await canvas.boundingBox();
    expect(canvasBox).not.toBeNull();
    expect(canvasBox?.width).toBeGreaterThan(0);
    expect(canvasBox?.height).toBeGreaterThan(0);

    // Check that no critical errors occurred
    const criticalErrors = errors.filter(
      (error) =>
        error.includes('WebGL') ||
        error.includes('Three') ||
        error.includes('Failed to compile')
    );
    expect(criticalErrors).toHaveLength(0);
  });

  // Additional test: Verify all key elements are present
  test('should display all key landing page elements', async ({ page }) => {
    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Check HUMARIDE heading
    const heading = page.locator('.landing-title');
    await expect(heading).toBeVisible();
    await expect(heading).toHaveText('HUMARIDE');

    // Check subheading
    const subheading = page.locator('.landing-subtitle');
    await expect(subheading).toBeVisible();
    await expect(subheading).toContainText('Physical AI & Humanoid Robotics');

    // Check Three.js canvas
    const canvas = page.locator('canvas');
    await expect(canvas).toBeVisible();

    // Check Read Book button
    const ctaButton = page.locator('.landing-cta-button');
    await expect(ctaButton).toBeVisible();
    await expect(ctaButton).toHaveText('Read Book');

    // Check all 4 stats cards
    const statsCards = page.locator('.ocean-card');
    await expect(statsCards).toHaveCount(4);

    // Verify each stats card is visible
    for (let i = 0; i < 4; i++) {
      await expect(statsCards.nth(i)).toBeVisible();
    }
  });

  // Additional test: Verify responsive layout
  test('should be responsive on mobile viewport', async ({ page }) => {
    // Set mobile viewport (iPhone 12 dimensions)
    await page.setViewportSize({ width: 390, height: 844 });

    // Wait for page to load
    await page.goto(BASE_URL);
    await page.waitForLoadState('networkidle');

    // Wait for canvas to load
    await page.waitForSelector('canvas', { timeout: 5000 });
    await page.waitForTimeout(1000);

    // Take mobile screenshot
    await expect(page).toHaveScreenshot('landing-page-mobile.png', {
      fullPage: true,
      animations: 'disabled',
    });

    // Verify elements are still visible on mobile
    await expect(page.locator('.landing-title')).toBeVisible();
    await expect(page.locator('.landing-subtitle')).toBeVisible();
    await expect(page.locator('canvas')).toBeVisible();
    await expect(page.locator('.landing-cta-button')).toBeVisible();
  });
});

/**
 * Test Checklist (T037-T041):
 * - [x] T037: Created Playwright test file at tests/visual-regression/landing-page.spec.ts
 * - [x] T038: Full-page screenshot baseline test
 * - [x] T039: Stats card hover state screenshot test
 * - [x] T040: Three.js scene rendering verification test
 * - [ ] T041: Generate baseline screenshots with `npx playwright test --update-snapshots` and commit to Git
 */
