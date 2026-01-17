import React, { useState, FormEvent } from "react";
import { useHistory } from "@docusaurus/router";
import Translate, { translate } from "@docusaurus/Translate";
import { useAuth } from "../../contexts/AuthContext";
import styles from "../../components/AuthForms/styles.module.css";

export default function SignupPage(): React.ReactElement {
  const [displayName, setDisplayName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [confirmPassword, setConfirmPassword] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [validationError, setValidationError] = useState<string | null>(null);
  const { signUpWithEmail, signInWithGoogle, error, clearError } = useAuth();
  const history = useHistory();

  const handleSignUp = async (e: FormEvent) => {
    e.preventDefault();
    setValidationError(null);
    clearError();

    // Validation
    if (password !== confirmPassword) {
      setValidationError("Passwords do not match");
      return;
    }

    if (password.length < 6) {
      setValidationError("Password must be at least 6 characters");
      return;
    }

    setIsLoading(true);

    try {
      await signUpWithEmail(email, password, displayName);
      history.push("/");
    } catch (err) {
      // Error is handled by AuthContext
    } finally {
      setIsLoading(false);
    }
  };

  const handleGoogleSignIn = async () => {
    setIsLoading(true);
    clearError();

    try {
      await signInWithGoogle();
      history.push("/");
    } catch (err) {
      // Error is handled by AuthContext
    } finally {
      setIsLoading(false);
    }
  };

  const displayError = validationError || error;

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h1 className={styles.authTitle}>
          <Translate id="auth.signup.title">Create Account</Translate>
        </h1>
        <p className={styles.authSubtitle}>
          <Translate id="auth.signup.subtitle">
            Join us to access the complete Physical AI Textbook.
          </Translate>
        </p>

        {displayError && <div className={styles.error}>{displayError}</div>}

        <button
          type="button"
          className={styles.googleButton}
          onClick={handleGoogleSignIn}
          disabled={isLoading}
        >
          <svg className={styles.googleIcon} viewBox="0 0 24 24">
            <path
              fill="#4285F4"
              d="M22.56 12.25c0-.78-.07-1.53-.2-2.25H12v4.26h5.92c-.26 1.37-1.04 2.53-2.21 3.31v2.77h3.57c2.08-1.92 3.28-4.74 3.28-8.09z"
            />
            <path
              fill="#34A853"
              d="M12 23c2.97 0 5.46-.98 7.28-2.66l-3.57-2.77c-.98.66-2.23 1.06-3.71 1.06-2.86 0-5.29-1.93-6.16-4.53H2.18v2.84C3.99 20.53 7.7 23 12 23z"
            />
            <path
              fill="#FBBC05"
              d="M5.84 14.09c-.22-.66-.35-1.36-.35-2.09s.13-1.43.35-2.09V7.07H2.18C1.43 8.55 1 10.22 1 12s.43 3.45 1.18 4.93l2.85-2.22.81-.62z"
            />
            <path
              fill="#EA4335"
              d="M12 5.38c1.62 0 3.06.56 4.21 1.64l3.15-3.15C17.45 2.09 14.97 1 12 1 7.7 1 3.99 3.47 2.18 7.07l3.66 2.84c.87-2.6 3.3-4.53 6.16-4.53z"
            />
          </svg>
          <Translate id="auth.signup.googleButton">Sign up with Google</Translate>
        </button>

        <div className={styles.divider}>
          <div className={styles.dividerLine}></div>
          <span className={styles.dividerText}>
            <Translate id="auth.signup.or">or</Translate>
          </span>
          <div className={styles.dividerLine}></div>
        </div>

        <form className={styles.form} onSubmit={handleSignUp}>
          <div className={styles.formGroup}>
            <label className={styles.label} htmlFor="displayName">
              <Translate id="auth.signup.nameLabel">Full Name</Translate>
            </label>
            <input
              id="displayName"
              type="text"
              className={styles.input}
              value={displayName}
              onChange={(e) => setDisplayName(e.target.value)}
              placeholder={translate({
                id: "auth.signup.namePlaceholder",
                message: "Enter your name",
              })}
              required
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label} htmlFor="email">
              <Translate id="auth.signup.emailLabel">Email</Translate>
            </label>
            <input
              id="email"
              type="email"
              className={styles.input}
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder={translate({
                id: "auth.signup.emailPlaceholder",
                message: "Enter your email",
              })}
              required
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label} htmlFor="password">
              <Translate id="auth.signup.passwordLabel">Password</Translate>
            </label>
            <input
              id="password"
              type="password"
              className={styles.input}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder={translate({
                id: "auth.signup.passwordPlaceholder",
                message: "Create a password (min 6 characters)",
              })}
              required
              minLength={6}
              disabled={isLoading}
            />
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label} htmlFor="confirmPassword">
              <Translate id="auth.signup.confirmPasswordLabel">Confirm Password</Translate>
            </label>
            <input
              id="confirmPassword"
              type="password"
              className={styles.input}
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder={translate({
                id: "auth.signup.confirmPasswordPlaceholder",
                message: "Confirm your password",
              })}
              required
              disabled={isLoading}
            />
          </div>

          <button type="submit" className={styles.submitButton} disabled={isLoading}>
            {isLoading ? (
              <span className={styles.spinner}></span>
            ) : (
              <Translate id="auth.signup.submitButton">Create Account</Translate>
            )}
          </button>
        </form>

        <div className={styles.links}>
          <span>
            <Translate id="auth.signup.hasAccount">Already have an account?</Translate>{" "}
            <a href="/auth/login" className={styles.link}>
              <Translate id="auth.signup.signInLink">Sign in</Translate>
            </a>
          </span>
        </div>
      </div>
    </div>
  );
}
