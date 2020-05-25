namespace Samples
{
    partial class Pendulum
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.FormTimer = new System.Windows.Forms.Timer(this.components);
            this.SuspendLayout();
            // 
            // FormTimer
            // 
            this.FormTimer.Tick += new System.EventHandler(this.FormTimer_Tick);
            // 
            // Pendulum
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(800, 450);
            this.Name = "Pendulum";
            this.Text = "Pendulum";
            this.Paint += new System.Windows.Forms.PaintEventHandler(this.Pendulum_Paint);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Timer FormTimer;
    }
}