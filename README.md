# Coela.dev Website

[![CI Build]()]()[![pages-build-deployment]()]()



Welcome to the repository for [coela.dev](https://ceola.dev/)-the portfolio site of **Coela Can't**.

---

## Table of Contents
- [About the Site](#about-the-site)
- [Technologies](#technologies)
- [Local Development](#local-development)
- [Continuous Integration (CI)](#continuous-integration-ci)
- [File/Folder Structure](#filefolder-structure)

---

## About the Site
This GitHub Pages site is a personal portfolio website
- Details on active and previous projects
- Contact information

---

## Technologies
- **[Jekyll](https://jekyllrb.com/):** A Ruby-based static site generator used to build GitHub Pages.
- **[GitHub Actions](https://docs.github.com/en/actions):** Automated workflows to check, build, and deploy the site.
- **Ruby Gems:** 
  - [w3c_validators](https://github.com/sparklemotion/w3c_validators) for HTML/CSS validation
  - [html-proofer](https://github.com/gjtorikian/html-proofer) for link checking
  - [rubocop](https://rubocop.org/) for Ruby style checks (optional, if you have Ruby code)

---

## Local Development

1. **Clone the Repo**  
   ```bash
   git clone https://github.com/coelacant1/coela.dev.git
   cd coela.dev
   ```

2. **Install Dependencies**
    Make sure you have Ruby installed (version 3.3+ recommended), then run:
    ```bash
    gem install bundle
    bundle install
    ```
    This installs all needed gems (Jekyll, etc.).

3. **Build & Serve Locally**
    ```bash
    bundler exec jekyll serve
    ```

4. **Running Tests/Checks**
    The script/cibuild (or equivalent) may include steps such as:
    ```bash
    #!/bin/sh
    set -e

    # Build the site
    bundler exec jekyll build

    # Check for broken links
    bundler exec htmlproofer ./_site

    # Run RuboCop (if configured)
    bundler exec rubocop -D

    # Validate HTML/CSS
    bundler exec script/validate-html
    ```

---

## Continuous Integration (CI)
GitHub Actions workflow (e.g. ci.yaml):
- Runs on push or pull_request.
- Checks out the code, sets up Ruby/Node, installs dependencies, and runs script/cibuild.
- If the build or validation fails, the workflow reports an error.

---

## File/Folder Structure
- _config.yml - Main Jekyll configuration.
- _sass/ - SCSS partials.
- _site/ - Generated build output (should not be committed if .gitignore is set).
- assets/ - Images, stylesheets, JavaScript, etc.
- script/ - Custom scripts (e.g., cibuild, validate-html).
- ci.yaml (in .github/workflows/) - The GitHub Actions workflow definition.
- Gemfile - Ruby dependencies.
